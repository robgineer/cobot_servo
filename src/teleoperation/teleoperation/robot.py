import numpy as np

from control_msgs.msg import JointJog
from moveit_msgs.msg import ServoStatus
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from py_utils.planner_utils import RobotPlanner
from py_utils.filter_utils import (
    KalmanFilter,
    SmoothVelocityCalculator,
    KalmanFilterState,
)

from py_utils.controller_types import ControlType, ControllerState


class Robot(object):
    """
    Primary functionalities:
        1. reads input created by the hand_tracker (x, y, command)
        2. applies an alpha filter to the x and y values
        3. derives a lateral and longitudinal velocity from the smooth x,y values
        4. applies a Kalman Filter on the velocities
        5. depending on the "command" received by the hand tracker, either forwards a:
           a. twist to MoveIt servo (command: follow)
           b. joint jog to MoveIt servo (command: rotate_joint_{x})
           c. open / close request to the move group (command: open/close)
           d. forward kinematic request (command: reset)
    """

    def __init__(
        self,
        twist_publisher,
        joint_publisher,
        command_type_client,
        logger,
        robot_name,
        arm_group,
        gripper_group,
        joint_prefix,
        frame_id,
    ):
        # delta twist publisher for MoveIt servo
        self.twist_publisher = twist_publisher

        # delta joint velocity publisher for MoveIt servo
        self.joint_publisher = joint_publisher

        # define twist command type
        self.command_type_client = command_type_client
        self.change_servo_command_type(ServoCommandType.Request.TWIST)

        self.logger = logger

        self.joint_prefix = joint_prefix

        self.frame_id = frame_id

        self.initialize_kalman_filter()

        self.last_active_servo_command = ControlType.TWIST

        self.vx_calculator = SmoothVelocityCalculator()
        self.vy_calculator = SmoothVelocityCalculator()

        # create MoveItPy instance
        self.arm = RobotPlanner(
            self.logger, robot_name, arm_group, gripper_group, self.joint_prefix
        )

        # initialize the hand tracker date to be processed
        self.last_received_control_info = None

        # self.current_controller_state = ControllerState()

        self.current_gripper_state = ControllerState.INACTIVE

    def initialize_kalman_filter(self):
        """
        Initialize the two state Kalman Filter
        state 1: long. velocity (vx)
        state 2: lat. velocity (vy)
        """
        P = np.array([[5, 0, 0, 0], [0, 5, 0, 0], [0, 0, 5, 0], [0, 0, 0, 5]])

        F = np.array([[1, 0.1, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.1], [0, 0, 0, 1]])

        Q = np.array(
            [[0, 0.01, 0, 0], [0.01, 0.01, 0, 0], [0, 0, 0, 0.01], [0, 0, 0.01, 0.01]]
        )
        # since the measurements are very noisy, we do not "trust" them
        # => higher values in measurement noise matrix
        R = np.array([[10, 0], [0, 10]])

        H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

        # todo: init KF when recognition is lost
        self.kf = KalmanFilter(P, F, R, Q, H, init_state=[0, 0, 0, 0], num_states=4)

    def reset_servo_command(self, timestamp):
        msg = TwistStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(0.0)
        msg.twist.linear.z = float(0.0)
        self.twist_publisher.publish(msg)

    def update_kalman_filter(self, x, y, current_time):
        """
        1. derives a lateral and longitudinal velocity from the smooth x,y values
        2. runs the update step of the Kalman Filter
        """
        # derive smooth velocities
        vx_raw = self.vx_calculator.calculate_velocity_from_signal(x, current_time)
        vy_raw = self.vy_calculator.calculate_velocity_from_signal(y, current_time)
        # run update step of kf
        self.kf.update([vx_raw, vy_raw])

    def predict_kalman_filter(self):
        """Run the predict step of the Kalman Filter."""
        self.kf.predict()

    def kalman_filter_values_are_non_zero(self):
        """Check if the values are non-zero."""
        return (
            self.kf.x[KalmanFilterState.VX] != 0.0
            and self.kf.x[KalmanFilterState.VY] != 0.0
        )

    def send_controls_to_robot(self, received_control_info, timestamp):
        """
        Process of the received message from the hand tracker and either:
        a. send a TWIST command to MoveIt servo
        b. send a JOINT_JOG command to MoveIt servo
        c. reset planner joints (go_home) via MoveItPy
        """
        control_info = received_control_info
        if control_info == None and self.last_received_control_info == None:
            # we have not received anything yet
            # no need to send out controls
            return 1
        elif control_info == None:
            # updates of received_hand_tracker_data are asynchronous
            # for servo, we need to keep sending KF predicted values
            # => use last servo control info in case no new hand tracker data was received
            if self.last_received_control_info.is_servo_type():
                control_info = self.last_received_control_info
            else:
                return 1

        if control_info.control_type == ControlType.TWIST:
            if (
                self.last_received_control_info != None
                and self.last_received_control_info.control_type != ControlType.TWIST
            ):
                # change command type to twist
                self.change_servo_command_type(ServoCommandType.Request.TWIST)
                self.initialize_kalman_filter()

            msg = TwistStamped()
            msg.header.stamp = timestamp
            msg.header.frame_id = self.frame_id
            velocity_scalar = 3.0
            msg.twist.linear.x = self.kf.x[KalmanFilterState.VX] * velocity_scalar
            msg.twist.linear.z = self.kf.x[KalmanFilterState.VY] * -velocity_scalar
            self.twist_publisher.publish(msg)

        elif control_info.control_type == ControlType.JOINT_JOG:
            if (
                self.last_received_control_info != None
                and self.last_received_control_info.control_type
                != ControlType.JOINT_JOG
            ):
                # change command type to joint jog
                self.change_servo_command_type(ServoCommandType.Request.JOINT_JOG)
                self.initialize_kalman_filter()

            joint_for_jog = self.joint_prefix + str(control_info.joint_jog_label.value)
            msg = JointJog()
            msg.header.stamp = timestamp
            msg.header.frame_id = self.frame_id
            msg.joint_names = [joint_for_jog]
            velocity_scalar = -7.0
            msg.velocities = [self.kf.x[KalmanFilterState.VY] * velocity_scalar]
            self.joint_publisher.publish(msg)

        elif control_info.control_type == ControlType.RESET:
            self.arm.go_home()

        """ # inactive at the moment
            elif control_info.control_type == ControlType.GRIPPER:
            print("GRIPPER ACTION")
            # check current state and execute opposite action
            if self.current_gripper_state == ControllerState.ACTIVE:
                self.arm.deactivate_gripper()
                self.current_gripper_state == ControllerState.INACTIVE
            elif self.current_gripper_state == ControllerState.INACTIVE:
                self.arm.activate_gripper()
                self.current_gripper_state == ControllerState.ACTIVE """

        self.last_received_control_info = control_info

    def change_servo_command_type(self, command_type):
        """Change servo command. TWIST vs. JOINT_JOG."""

        req = ServoCommandType.Request()
        req.command_type = command_type
        future = self.command_type_client.call_async(req)

        def callback_complete_(fut):
            """Inline function for callback."""
            response = fut.result()
            if response.success:
                self.logger.info("Switched to servo mode: " + str(command_type))
            else:
                self.logger.error("Failed to switch servo mode!")

        future.add_done_callback(callback_complete_)

    def execute_robot_fallback(self, msg):
        """Execute auto fallback."""

        status_codes_forcing_reset = [
            ServoStatus.DECELERATE_FOR_APPROACHING_SINGULARITY,
            ServoStatus.HALT_FOR_SINGULARITY,
            ServoStatus.DECELERATE_FOR_COLLISION,
            ServoStatus.HALT_FOR_COLLISION,
            ServoStatus.JOINT_BOUND,
        ]

        if msg.code in status_codes_forcing_reset:
            self.logger.error(
                "----- Arm approaching singularity / collision / joint bounds -----"
            )
            self.logger.error("## forcing robot reset ##")
            self.arm.go_home()  # execute fallback joint configuration
