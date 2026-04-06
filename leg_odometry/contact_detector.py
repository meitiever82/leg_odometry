"""接触检测：踝关节力矩阈值法 + 滞后。"""

import numpy as np


class ContactDetector:
    """基于踝关节力矩的接触检测器（带滞后）。

    当 |effort| > threshold + hysteresis 时判定为接触，
    当 |effort| < threshold - hysteresis 时判定为腾空，
    中间区域保持上一次状态。
    """

    def __init__(self, threshold: float = 5.0, hysteresis: float = 1.0,
                 fk_z_threshold: float = 0.0):
        self.threshold = threshold
        self.hysteresis = hysteresis
        self.fk_z_threshold = fk_z_threshold  # FK Z 辅助 (0=关闭)
        self._contact_left = False
        self._contact_right = False

    def update(self, effort_left: float, effort_right: float,
               fk_z_left: float = None, fk_z_right: float = None) -> tuple[bool, bool]:
        """更新接触状态。

        Args:
            effort_left:  左踝关节力矩 (Nm)
            effort_right: 右踝关节力矩 (Nm)
            fk_z_left: 左脚 FK Z 坐标 (body 系，可选)
            fk_z_right: 右脚 FK Z 坐标 (body 系，可选)

        Returns:
            (left_contact, right_contact)
        """
        self._contact_left = self._detect(
            abs(effort_left), self._contact_left, fk_z_left)
        self._contact_right = self._detect(
            abs(effort_right), self._contact_right, fk_z_right)
        return self._contact_left, self._contact_right

    def _detect(self, effort_abs: float, prev_contact: bool,
                fk_z: float = None) -> bool:
        upper = self.threshold + self.hysteresis
        lower = self.threshold - self.hysteresis

        # 基本力矩检测
        if effort_abs > upper:
            result = True
        elif effort_abs < lower:
            result = False
        else:
            result = prev_contact

        # FK Z 辅助: 脚低 = 更可能着地，滞后区间内优先判定着地
        if self.fk_z_threshold != 0 and fk_z is not None:
            if not result and prev_contact and fk_z < self.fk_z_threshold:
                # 力矩说腾空，但脚还很低 → 延迟判定为腾空
                result = True

        return result

    @property
    def left_contact(self) -> bool:
        return self._contact_left

    @property
    def right_contact(self) -> bool:
        return self._contact_right
