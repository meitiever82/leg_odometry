"""AIRY 雷达点云(x,y,z,intensity,ring,timestamp)sidecar 发布节点。

Isaac 主程序(isaac/w2_sim_app.py)跑在 cp311 嵌入式 Python 里,无法干净地用
系统 cp312 rclpy 直接发 PointCloud2(双 ROS 安装 ABI 冲突,详见 app 内注释)。
故 Isaac 侧已把每帧 6 字段交错打包成真实 AIRY 的 26 字节/点布局,经 UNIX 域 socket
(SOCK_STREAM、长度前缀分帧)推过来;本节点跑在干净的系统 ROS2 环境,直接把该原始
载荷塞进 sensor_msgs/PointCloud2.data 发 /rslidar_points(无需重排,字节布局已对齐)。

帧格式(与 app 内 _LIDAR_HDR 一致,小端):
    magic(u32=0x52534C44) seq(u32) sim_time(f64) n_points(u32) point_step(u32)
    之后紧跟 n*point_step 字节的已交错载荷,每点 26 字节:
        x f32@0  y f32@4  z f32@8  intensity f32@12  ring u16@16  timestamp f64@18。
字段对齐真实 AIRY(point_step=26):ring 是 AIRY 96 线的线号(0..95,= emitterId),
timestamp 是绝对 sim-time(FLOAT64,= 帧 header + 圈内偏移,与 IMU/相机同 sim 钟,
首点≈header.stamp)。这两个字段已由 app 算好,本节点不做任何加工。

帧 header.stamp:用 Isaac 传来的 sim_time(秒),与 /clock、相机、imu 对齐。

socket 路径默认 /tmp/w2_sim_lidar.sock,可用环境变量 W2_LIDAR_SOCK 覆盖。
"""
import os
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

SOCK_PATH = os.environ.get("W2_LIDAR_SOCK", "/tmp/w2_sim_lidar.sock")
HDR = struct.Struct("<IIdII")  # magic, seq, sim_time, n_points, point_step
MAGIC = 0x52534C44
POINT_STEP = 26  # 4 个 float32 + 1 个 uint16 + 1 个 float64,对齐真实 AIRY

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]


def _recv_exact(conn, n):
    """从 stream socket 精确读 n 字节;EOF 返回 None。"""
    chunks = []
    got = 0
    while got < n:
        b = conn.recv(n - got)
        if not b:
            return None
        chunks.append(b)
        got += len(b)
    return b"".join(chunks)


class LidarPC2Publisher(Node):
    def __init__(self):
        super().__init__("lidar_pc2_publisher")
        self.declare_parameter("topic", "/rslidar_points")
        self.declare_parameter("frame_id", "rslidar")
        self.declare_parameter("sock_path", SOCK_PATH)
        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.sock_path = self.get_parameter("sock_path").value
        # RELIABLE/KEEP_LAST(10):匹配 Isaac ROS2RtxLidarHelper 的默认 QoS,确保
        # 录包时不丢帧。实测:~85k 点 ≈1.36MB 的大消息走 BEST_EFFORT 会被 DDS 大量
        # 丢弃(topic hz 仅 ~1.5Hz,录包只剩 ~32% 帧);改 RELIABLE 后 recorder 拿全
        # (旧的 XYZ-only helper 用的就是默认 RELIABLE,录包 ~100% 帧、header 10Hz)。
        # 下游 rtabmap 用 qos:=2(BEST_EFFORT)订阅 RELIABLE publisher 仍兼容
        # (BEST_EFFORT sub 可收 RELIABLE pub)。
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.count = 0
        self._stop = threading.Event()
        # 先删旧 socket 文件再监听(上次异常退出可能残留)。
        try:
            os.unlink(self.sock_path)
        except FileNotFoundError:
            pass
        self.srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.srv.bind(self.sock_path)
        self.srv.listen(1)
        self.srv.settimeout(1.0)
        self.get_logger().info(f"listening on {self.sock_path}, publishing {topic}")
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.get_logger().info("producer connected")
            conn.settimeout(5.0)
            try:
                self._handle_conn(conn)
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                self.get_logger().warn(f"connection error: {e}")
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
            self.get_logger().info("producer disconnected, waiting for reconnect")

    def _handle_conn(self, conn):
        while not self._stop.is_set() and rclpy.ok():
            hdr = _recv_exact(conn, HDR.size)
            if hdr is None:
                return
            magic, seq, sim_time, n, point_step = HDR.unpack(hdr)
            if magic != MAGIC:
                self.get_logger().error(f"bad magic {magic:#x}, dropping connection")
                return
            if point_step != POINT_STEP:
                self.get_logger().error(
                    f"point_step mismatch: got {point_step}, expected {POINT_STEP}; "
                    "dropping connection")
                return
            payload = _recv_exact(conn, n * point_step)
            if payload is None:
                return
            self._publish(sim_time, n, payload)

    def _publish(self, sim_time, n, payload):
        msg = PointCloud2()
        msg.header = Header()
        sec = int(sim_time)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int(round((sim_time - sec) * 1e9))
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n
        msg.fields = FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * n
        msg.data = payload
        msg.is_dense = True
        self.pub.publish(msg)
        self.count += 1
        if self.count <= 3 or self.count % 50 == 0:
            self.get_logger().info(f"published msg #{self.count} n={n} t={sim_time:.3f}")

    def destroy_node(self):
        self._stop.set()
        try:
            self.srv.close()
        except OSError:
            pass
        try:
            os.unlink(self.sock_path)
        except FileNotFoundError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarPC2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
