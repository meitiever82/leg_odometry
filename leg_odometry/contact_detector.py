"""接触检测：踝关节力矩阈值法 + 滞后。"""

import numpy as np


class ContactDetector:
    """基于踝关节力矩的接触检测器（带滞后）。

    当 |effort| > threshold + hysteresis 时判定为接触，
    当 |effort| < threshold - hysteresis 时判定为腾空，
    中间区域保持上一次状态。
    """

    def __init__(self, threshold: float = 5.0, hysteresis: float = 1.0):
        self.threshold = threshold
        self.hysteresis = hysteresis
        self._contact_left = False
        self._contact_right = False

    def update(self, effort_left: float, effort_right: float) -> tuple[bool, bool]:
        """更新接触状态。

        Args:
            effort_left:  左踝关节力矩 (Nm)
            effort_right: 右踝关节力矩 (Nm)

        Returns:
            (left_contact, right_contact)
        """
        self._contact_left = self._detect(
            abs(effort_left), self._contact_left)
        self._contact_right = self._detect(
            abs(effort_right), self._contact_right)
        return self._contact_left, self._contact_right

    def _detect(self, effort_abs: float, prev_contact: bool) -> bool:
        upper = self.threshold + self.hysteresis
        lower = self.threshold - self.hysteresis
        if effort_abs > upper:
            return True
        elif effort_abs < lower:
            return False
        else:
            return prev_contact

    @property
    def left_contact(self) -> bool:
        return self._contact_left

    @property
    def right_contact(self) -> bool:
        return self._contact_right
