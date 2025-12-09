"""
Planner specific utilities.
"""

import time
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from moveit.core.kinematic_constraints import construct_joint_constraint

from sensor_msgs.msg import JointState


class RobotPlanner(object):

    def __init__(
        self, logger, robot_name, arm_group_name, gripper_group_name, joint_prefix
    ):

        self.moveit_config = generate_moveit_config()
        self.logger = logger
        self.arm_group_name = arm_group_name
        self.gripper_group_name = gripper_group_name
        self.joint_prefix = joint_prefix

        wait_for_joint_states(self.logger)

        max_attempts = 100
        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            try:
                self.robot = MoveItPy(
                    node_name=robot_name,
                    config_dict=self.moveit_config,
                )
                break
            except Exception as e:
                self.logger.warning(
                    f"[MoveItPy] Attempt {attempt} failed: {e.__class__.__name__}: {e}"
                )
                if attempt == max_attempts:
                    self.logger.error("[MoveItPy] All attempts failed. Giving up.")
                raise  # re-raise the last exception

        self.robot_arm = self.robot.get_planning_component(arm_group_name)
        self.robot_gripper = self.robot.get_planning_component(gripper_group_name)
        self.logger.info("MoveItPy instance created for " + str(robot_name))

    def go_home(self, sleep_time=0.0):
        """Setup and execution of forward kinematics for the robot arm.
           In MoveItPy the fk is achieved using joint constraints.

        Returns true if a plan was successful.
        """
        # define the current state
        self.robot_arm.set_start_state_to_current_state()
        robot_model = self.robot.get_robot_model()
        robot_state = RobotState(robot_model)

        # joint values for home state
        joint_values = {
            self.joint_prefix + str("joint_1"): -1.57,
            self.joint_prefix + str("joint_2"): 0.76,
            self.joint_prefix + str("joint_3"): -0.76,
            self.joint_prefix + str("joint_4"): 0.0,
            self.joint_prefix + str("joint_5"): -0.76,
            self.joint_prefix + str("joint_6"): 0.0,
        }
        # define joint constraints
        robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=self.robot.get_robot_model().get_joint_model_group(
                self.arm_group_name
            ),
        )
        self.robot_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        plan_result = self.robot_arm.plan()

        if plan_result:
            self.logger.info("Planning successful")
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
            time.sleep(sleep_time)
        else:
            self.logger.error("Planning failed")

        return plan_result


def wait_for_joint_states(logger):
    """Creates a node, subscribes to /joint_states and waits until
    joint states are available. Without waiting, we could run into a
    'RuntimeError: Unable to configure planning scene monitor'.

    Args:
        logger: the logger instance of the caller
    """
    node = rclpy.create_node("wait_for_joint_states")

    joint_states_msg = {"data": None}

    node.create_subscription(
        JointState,
        "/joint_states",
        lambda msg: joint_states_msg.update({"data": msg}),
        10,
    )

    while joint_states_msg["data"] is None:
        logger.warn("Joint states are not available. Retrying...")
        rclpy.spin_once(node, timeout_sec=0.1)

    logger.info("Joint states received")

    node.destroy_node()


def generate_moveit_config() -> dict:
    """Generates a MoveIt2 configuration using the MoveItConfigsBuilder.

    Unlike the C++ MoveGroup API, the Python MoveIt2 API requires configuration.
    We basically need most of the config files required to start the move group here again.
    It is also close to impossible to create a config that is accepted by MoveItPy
    without the MoveItConfigsBuilder (hence, the config creation differs from the launch files).

    Returns: MoveIt2 config as dict.
    """

    moveit_config = (
        MoveItConfigsBuilder(robot_name="dual_cobot")
        .robot_description(
            file_path=get_package_share_directory("dual_cobot_model")
            + "/urdf/robot_model.urdf.xacro"
        )
        .robot_description_semantic(
            file_path=get_package_share_directory("dual_cobot_moveit_config")
            + "/config/dual_cobot_model.srdf"
        )
        .robot_description_kinematics(
            file_path=get_package_share_directory("dual_cobot_moveit_config")
            + "/config/kinematics.yaml"
        )
        .joint_limits(
            file_path=get_package_share_directory("dual_cobot_moveit_config")
            + "/config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=get_package_share_directory("dual_cobot_moveit_config")
            + "/config/moveit_controllers.yaml"
        )
        # apparently this file is read based on convention
        # in <robot_name>_moveit_config
        # .planning_pipelines(
        #    file_path=get_package_share_directory("dual_cobot_moveit_config")
        #    + "/config/ompl_planning.yaml"
        # )
        .pilz_cartesian_limits(
            file_path=get_package_share_directory("dual_cobot_moveit_config")
            + "/config/pilz_cartesian_limits.yaml"
        )
        # this defines the planner config (unclear why ompl_planning.yaml is not enough)
        .moveit_cpp(
            file_path=get_package_share_directory("dual_cobot_moveit_config")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
        .to_dict()
    )
    return moveit_config
