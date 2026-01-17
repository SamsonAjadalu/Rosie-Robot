#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
# moveit python library
from tf2_ros import Buffer, TransformListener, TransformException
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory

        robot_trajectory_msg = plan_result.trajectory.get_robot_trajectory_msg()
        logger.info(f"Robot Trajectory Message: {robot_trajectory_msg}")

        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

class Controller(Node):

    def __init__(self):
        super().__init__('commander')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_point',
            self.listener_callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(Bool, "gripper_bool", 10)
        self.bool_msg = Bool()
        self.bool_msg.data = False
        

        # Initialize PoseStamped for goal
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "base_link"  # Replace "panda_link0" with your robot's base frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Instantiate MoveItPy instance and get planning components
        self.robot = MoveItPy(node_name="moveit_py")
        self.right_arm = self.robot.get_planning_component("right_arm")  # Replace with your arm group
        self.right_gripper = self.robot.get_planning_component("right_gripper")  # Replace with your gripper group
        self.logger = get_logger("moveit_py.pose_goal")

        # Get robot model and state
        robot_model = self.robot.get_robot_model()
        self.robot_state = RobotState(robot_model)

        self.logger.info("Commander initialized with right_arm and right_gripper.")
        self.update_h = 0.05
        self.gripper_offset = 0.07 + self.update_h
        self.carry_clearance  = 0.15 + self.update_h
        self.approach_clearance = 0.15 + self.update_h

# Function to move to a specific pose
    def move_to(self, x, y, z, xo, yo, zo, wo):
        """
        Move the robot arm to a specific pose.
        """
        self.pose_goal.pose.position.x = x
        self.pose_goal.pose.position.y = y
        self.pose_goal.pose.position.z = z
        self.pose_goal.pose.orientation.x = xo
        self.pose_goal.pose.orientation.y = yo
        self.pose_goal.pose.orientation.z = zo
        self.pose_goal.pose.orientation.w = wo

        # Set goal state for the right_arm
        self.right_arm.set_goal_state(pose_stamped_msg=self.pose_goal, pose_link="right_gripper_base_link")  # Adjust `pose_link` if necessary
        plan_and_execute(self.robot, self.right_arm, self.logger, sleep_time=5.0)

    # Function for gripper actions
    def gripper_action(self, action):
        """
        Open or close the gripper.
        """
        self.right_gripper.set_start_state_to_current_state()

        if action == 'close':
            joint_values = {"right_finger_joint": 0.3}  # Replace with actual joint name and open position
            self.bool_msg.data = True  # Set the boolean value

        elif action == 'open':
            joint_values = {"right_finger_joint": 0.01}  # Replace with actual joint name and closed position
            self.bool_msg.data = False  # Set the boolean value
            
        else:
            self.logger.info("Invalid gripper action.")
            return
        self.publisher_.publish(self.bool_msg)  # Publish the message

        # Update joint positions and set goal state
        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.robot.get_robot_model().get_joint_model_group("right_gripper"),  # Adjust group name as needed
        )
        self.right_gripper.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_and_execute(self.robot, self.right_gripper, self.logger, sleep_time=3.0)
    def listener_callback(self, msg):


        x = msg.data[0]
        y = msg.data[1]
        z = msg.data[2]

        qx = 0.982
        qy = 0.11
        qz = 0.0
        qw = 0.15

        self.get_logger().info(f" Msg.data object {msg.data}")

        self.move_to(x, y, z + self.approach_clearance, qx, qy, qz, qw)
        self.get_logger().info(f" first action")


        self.gripper_action("open")
        self.get_logger().info(f" second action")

        self.move_to(x, y, z + self.gripper_offset, qx, qy, qz, qw)
        self.get_logger().info(f" third action")

        self.gripper_action("close")
        self.get_logger().info(f" fourth action")

        self.move_to(x, y, z + self.carry_clearance, qx, qy, qz, qw)
        self.get_logger().info(f" fifth action")

        self.move_to(x - 0.2, y, z + self.carry_clearance, qx, qy, qz, qw)
        self.get_logger().info(f" sixth action")

        self.gripper_action("open")
        self.bool_msg.data = False
        self.publisher_.publish(self.bool_msg)  # Publish the message

        self.get_logger().info(f" seventh action")




    

if __name__ == '__main__':
    rclpy.init(args=None)

    controller = Controller()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
