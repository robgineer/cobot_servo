from enum import Enum
from enum import IntEnum


class HandLabel(IntEnum):
    LEFT = 0
    RIGHT = 1


class ControlType(Enum):
    RESET = 0
    TWIST = 1
    JOINT_JOG = 2
    GRIPPER = 3
    INVALID = 4


class ControllerState(Enum):
    INACTIVE = 0
    ACTIVE = 1


class JointJogLabel(Enum):
    """We are currently controlling only two joints via joint jog."""

    FIRST = "joint_5"
    SECOND = "joint_2"


class HandTrackerInfo(object):
    def __init__(self, control_type, joint_jog_label=None):
        self.control_type = control_type
        self.joint_jog_label = joint_jog_label

    def is_servo_type(self):
        return (
            self.control_type == ControlType.TWIST
            or self.control_type == ControlType.JOINT_JOG
        )


class HandTrackerData(object):
    def __init__(self):
        self.hand_label = None
        self.control_info = None  # HandTrackerInfo type
        self.x = None
        self.y = None

    def set_control_info(self, control_info):
        self.control_info = control_info

    def set_x_y_values(self, x, y):
        self.x = x
        self.y = y

    def set_hand_label(self, hand_label):
        self.hand_label = hand_label

    def has_values(self):
        """Check if object is initialized with valid data."""

        if (
            self.hand_label != None
            and self.control_info != None
            and self.x != None
            and self.y != None
        ):
            return True
        else:
            return False
