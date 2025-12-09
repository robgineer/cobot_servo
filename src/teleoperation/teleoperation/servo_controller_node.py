import time
from queue import Queue
from teleoperation.robot import Robot

from rclpy.node import Node
from control_msgs.msg import JointJog
from moveit_msgs.msg import ServoStatus
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from py_utils.plot_utils import LivePlotter

from py_utils.filter_utils import KalmanFilterState
from py_utils.controller_types import HandTrackerData, HandLabel, HandLabel


class ServoController(Node):
    """
    Control node for the dual Cobot.

    Primary functionalities:
        1. creates two Robot instances
        2. calls update function of Robots Kalman Filter on reception of a
           new message from the hand tracker
        3. if no new message is received, calls Robots predict function
        4. calls Robots send_controls_to_robot function
        5. in case a Robot is close to an error, triggers a fallback
    """

    def __init__(self, command_queue: Queue):
        super().__init__("servo_controller")

        # commands from image processing
        self.command_queue = command_queue

        # delta twist publisher for MoveIt servo
        left_twist_publisher = self.create_publisher(
            TwistStamped, "/left_arm/servo_node/delta_twist_cmds", 10
        )
        # delta joint velocity publisher for MoveIt servo
        left_joint_publisher = self.create_publisher(
            JointJog, "/left_arm/servo_node/delta_joint_cmds", 10
        )
        # define twist command type
        left_command_type_client = self.create_client(
            ServoCommandType, "left_arm/servo_node/switch_command_type"
        )
        left_command_type_client.wait_for_service(timeout_sec=2.0)
        # plotter
        self.left_live_plotter = LivePlotter()
        # robot instance
        self.left_robot = Robot(
            left_twist_publisher,
            left_joint_publisher,
            left_command_type_client,
            self.get_logger(),
            robot_name="left_cobot",
            arm_group="left_arm_group",
            gripper_group="left_vacuum_gripper_group",
            joint_prefix="left_",
            frame_id="left_base_link",
        )

        # MoveIt servo status topic
        self.left_servo_status_sub = self.create_subscription(
            ServoStatus,
            "/left_arm/servo_node/status",
            self.left_servo_status_callback,
            10,
        )

        # delta twist publisher for MoveIt servo
        right_twist_publisher = self.create_publisher(
            TwistStamped, "/right_arm/servo_node/delta_twist_cmds", 10
        )
        # delta joint velocity publisher for MoveIt servo
        right_joint_publisher = self.create_publisher(
            JointJog, "/right_arm/servo_node/delta_joint_cmds", 10
        )
        # define twist command type
        right_command_type_client = self.create_client(
            ServoCommandType, "right_arm/servo_node/switch_command_type"
        )
        right_command_type_client.wait_for_service(timeout_sec=2.0)

        self.right_live_plotter = LivePlotter()

        self.right_robot = Robot(
            right_twist_publisher,
            right_joint_publisher,
            right_command_type_client,
            self.get_logger(),
            robot_name="right_cobot",
            arm_group="right_arm_group",
            gripper_group="right_vacuum_gripper_group",
            joint_prefix="right_",
            frame_id="right_base_link",
        )

        # MoveIt servo status topic
        self.right_servo_status_sub = self.create_subscription(
            ServoStatus,
            "/right_arm/servo_node/status",
            self.right_servo_status_callback,
            10,
        )

        # run the timer callback with 30ms
        self.create_timer(0.03, self.timer_callback)

    def left_servo_status_callback(self, msg):
        """
        Trigger auto fallback for left_arm.

        In case the robot arm is close to an error state or in error state,
        we run an fk request to return to the home state.
        """
        self.left_robot.execute_robot_fallback(msg)

    def right_servo_status_callback(self, msg):
        """Trigger auto fallback for left_arm."""

        self.right_robot.execute_robot_fallback(msg)

    def timer_callback(self):
        """Main function."""

        current_time = time.time()

        received_left_hand_tracker_data = HandTrackerData()
        received_right_hand_tracker_data = HandTrackerData()

        if not self.command_queue.empty():
            # the hand tracker has updated the queue => update KF

            # get hand tracker data from queue
            all_hands_tracker = self.command_queue.get_nowait()
            received_left_hand_tracker_data = all_hands_tracker[HandLabel.LEFT]
            received_right_hand_tracker_data = all_hands_tracker[HandLabel.RIGHT]

            # the received data could be empty if there are no hands to be tracked in the video
            if received_left_hand_tracker_data.has_values():
                self.left_robot.update_kalman_filter(
                    received_left_hand_tracker_data.x,
                    received_left_hand_tracker_data.y,
                    current_time,
                )
            else:
                # there is nothing to track
                # => reset filter
                self.left_robot.initialize_kalman_filter()
                self.left_robot.reset_servo_command(self.get_clock().now().to_msg())

            if received_right_hand_tracker_data.has_values():
                self.right_robot.update_kalman_filter(
                    received_right_hand_tracker_data.x,
                    received_right_hand_tracker_data.y,
                    current_time,
                )
            else:
                self.right_robot.initialize_kalman_filter()
                self.right_robot.reset_servo_command(self.get_clock().now().to_msg())
        else:
            # we did not receive updated hand tracker data (the camera is slower than this thread)
            # run predict step to estimate (and smooth) new values
            # => async filtering
            self.left_robot.predict_kalman_filter()
            self.right_robot.predict_kalman_filter()

        # run plotter to visualize the signals over time
        self.left_live_plotter.update_plot(
            current_time,
            received_left_hand_tracker_data.x,
            received_left_hand_tracker_data.y,
            self.left_robot.kf.x[KalmanFilterState.VX],
            self.left_robot.kf.x[KalmanFilterState.VY],
        )
        self.right_live_plotter.update_plot(
            current_time,
            received_right_hand_tracker_data.x,
            received_right_hand_tracker_data.y,
            self.right_robot.kf.x[KalmanFilterState.VX],
            self.right_robot.kf.x[KalmanFilterState.VY],
        )

        # send cobot controls
        self.left_robot.send_controls_to_robot(
            received_left_hand_tracker_data.control_info,
            self.get_clock().now().to_msg(),
        )
        self.right_robot.send_controls_to_robot(
            received_right_hand_tracker_data.control_info,
            self.get_clock().now().to_msg(),
        )
