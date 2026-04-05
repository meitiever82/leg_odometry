"""URDF 运动学封装：从 URDF 加载运动学链，计算 FK。

使用 PyKDL + urdf_parser_py，手动构建 KDL Tree（不依赖 kdl_parser_py）。
"""

import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF


def _urdf_pose_to_kdl_frame(pose):
    """将 URDF 的 pose (xyz + rpy) 转为 KDL Frame。"""
    if pose is None:
        return PyKDL.Frame()
    xyz = pose.xyz if pose.xyz else [0, 0, 0]
    rpy = pose.rpy if pose.rpy else [0, 0, 0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
        PyKDL.Vector(xyz[0], xyz[1], xyz[2]),
    )


def _urdf_joint_to_kdl_joint(joint):
    """将 URDF joint 转为 KDL Joint。"""
    origin = _urdf_pose_to_kdl_frame(joint.origin)
    axis = joint.axis if joint.axis else [1, 0, 0]

    if joint.type == 'fixed':
        return origin, PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)
    elif joint.type in ('revolute', 'continuous'):
        return origin, PyKDL.Joint(
            joint.name,
            PyKDL.Vector(0, 0, 0),  # origin
            PyKDL.Vector(*axis),     # axis
            PyKDL.Joint.RotAxis,
        )
    elif joint.type == 'prismatic':
        return origin, PyKDL.Joint(
            joint.name,
            PyKDL.Vector(0, 0, 0),  # origin
            PyKDL.Vector(*axis),     # axis
            PyKDL.Joint.TransAxis,
        )
    else:
        return origin, PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)


def _build_kdl_tree(robot: URDF) -> PyKDL.Tree:
    """从 URDF 模型构建 KDL Tree。"""
    tree = PyKDL.Tree(robot.get_root())

    # 构建 parent -> [children joints] 映射
    parent_map = {}
    for joint in robot.joints:
        parent_map.setdefault(joint.parent, []).append(joint)

    def _add_children(parent_link_name):
        if parent_link_name not in parent_map:
            return
        for joint in parent_map[parent_link_name]:
            origin, kdl_joint = _urdf_joint_to_kdl_joint(joint)
            # 获取 child link 的 inertial（KDL 需要，但 FK 不用，给空的）
            segment = PyKDL.Segment(
                joint.child,
                kdl_joint,
                origin,
                PyKDL.RigidBodyInertia(),
            )
            tree.addSegment(segment, parent_link_name)
            _add_children(joint.child)

    _add_children(robot.get_root())
    return tree


class LegKinematics:
    """双腿正运动学计算器。

    使用 KDL 从 URDF 加载运动学链，计算 base_link 到足端的 FK。
    """

    def __init__(self, urdf_str: str, base_link: str,
                 left_foot_link: str, right_foot_link: str):
        robot = URDF.from_xml_string(urdf_str)
        tree = _build_kdl_tree(robot)

        # 左腿链
        self._chain_left = tree.getChain(base_link, left_foot_link)
        self._fk_left = PyKDL.ChainFkSolverPos_recursive(self._chain_left)

        # 右腿链
        self._chain_right = tree.getChain(base_link, right_foot_link)
        self._fk_right = PyKDL.ChainFkSolverPos_recursive(self._chain_right)

        # 提取链中关节名（按 KDL 顺序）
        self.left_joint_names = self._get_joint_names(self._chain_left)
        self.right_joint_names = self._get_joint_names(self._chain_right)

    @staticmethod
    def _get_joint_names(chain: PyKDL.Chain) -> list[str]:
        names = []
        for i in range(chain.getNrOfSegments()):
            joint = chain.getSegment(i).getJoint()
            if joint.getType() != PyKDL.Joint.Fixed:
                names.append(joint.getName())
        return names

    def fk_left(self, joint_positions: dict[str, float]) -> np.ndarray:
        """计算左脚在 base_link 系中的位置 [x, y, z]。"""
        return self._compute_fk(
            self._chain_left, self._fk_left,
            self.left_joint_names, joint_positions)

    def fk_right(self, joint_positions: dict[str, float]) -> np.ndarray:
        """计算右脚在 base_link 系中的位置 [x, y, z]。"""
        return self._compute_fk(
            self._chain_right, self._fk_right,
            self.right_joint_names, joint_positions)

    @staticmethod
    def _compute_fk(chain, fk_solver, joint_names, joint_positions):
        n = chain.getNrOfJoints()
        q = PyKDL.JntArray(n)
        for i, name in enumerate(joint_names):
            q[i] = joint_positions.get(name, 0.0)

        frame = PyKDL.Frame()
        fk_solver.JntToCart(q, frame)
        p = frame.p
        return np.array([p.x(), p.y(), p.z()])

    def fk_left_frame(self, joint_positions: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        """计算左脚完整位姿 (position, rotation_matrix)。"""
        return self._compute_fk_frame(
            self._chain_left, self._fk_left,
            self.left_joint_names, joint_positions)

    def fk_right_frame(self, joint_positions: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        """计算右脚完整位姿 (position, rotation_matrix)。"""
        return self._compute_fk_frame(
            self._chain_right, self._fk_right,
            self.right_joint_names, joint_positions)

    @staticmethod
    def _compute_fk_frame(chain, fk_solver, joint_names, joint_positions):
        n = chain.getNrOfJoints()
        q = PyKDL.JntArray(n)
        for i, name in enumerate(joint_names):
            q[i] = joint_positions.get(name, 0.0)

        frame = PyKDL.Frame()
        fk_solver.JntToCart(q, frame)
        p = frame.p
        R = frame.M
        pos = np.array([p.x(), p.y(), p.z()])
        rot = np.array([
            [R[0, 0], R[0, 1], R[0, 2]],
            [R[1, 0], R[1, 1], R[1, 2]],
            [R[2, 0], R[2, 1], R[2, 2]],
        ])
        return pos, rot
