#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
import yaml
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5 as robot
#from pymoveit2.robots import panda as robot
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Plane

def main():
    rclpy.init()

    #ADDED
    def load_yaml(file):
      with open(file, 'r') as file:
        yaml_data = yaml.safe_load(file)
      return yaml_data
    
    robot_type = "ur5"
    arm_params = load_yaml(
        "/home/jh/robot_ws/src/multi_robot_arm/config/ur/" + robot_type + "/kinematics.yaml"
    )

    # Create node for this example
    node1 = Node("ex_pose_goal", namespace=arm_params["arm1"]["name"])
    node2 = Node("ex_pose_goal", namespace=arm_params["arm2"]["name"])

    # Declare parameters for position and orientation
    node1.declare_parameter("position", [0.5, 0.0, 0.25])
    node1.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node1.declare_parameter("cartesian", False)
    node2.declare_parameter("position", [0.5, 0.0, 0.25])
    node2.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node2.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2_1 = MoveIt2(
        node=node1,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group
    )

    moveit2_2 = MoveIt2(
        node=node2,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node1)
    executor.add_node(node2)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    fixed_position_node1= [0.5-float(arm_params["arm1"]["x_pose"]), 0.0-float(arm_params["arm1"]["y_pose"]), 0.25-float(arm_params["arm1"]["Y"])]
    fixed_position_node2= [0.5-float(arm_params["arm2"]["x_pose"]), 0.0-float(arm_params["arm2"]["y_pose"]), 0.25-float(arm_params["arm2"]["Y"])]
    fixed_quat_xyzw_node1= [0.0, 0.0, 0.0, 1.0]
    fixed_quat_xyzw_node2= [0.0, 0.0, 0.0, 1.0]
    #각 node의 포지션을 설정
    position_node1=fixed_position_node1
    quat_xyzw_node1=fixed_quat_xyzw_node1
    position_node2=fixed_position_node2
    quat_xyzw_node2=fixed_quat_xyzw_node2
    cartesian=True

    '''
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    '''
    # Move to pose
    node1.get_logger().info(
        f"Moving to {{position: {list(position_node1)}, quat_xyzw: {list(quat_xyzw_node1)}}}"
    )
    node2.get_logger().info(
        f"Moving to {{position: {list(position_node2)}, quat_xyzw: {list(quat_xyzw_node2)}}}"
    )

    try:
        add_ground_plane(node1)
        add_ground_plane(node2)
        print(fixed_position_node1)
        print(fixed_position_node2)
        moveit2_1.move_to_pose(position=position_node1, quat_xyzw=quat_xyzw_node1, cartesian=cartesian)
        moveit2_2.move_to_pose(position=position_node2, quat_xyzw=quat_xyzw_node2, cartesian=cartesian)
        time.sleep(5.0)
        moveit2_1.wait_until_executed()
        moveit2_2.wait_until_executed()
        moveit2_1.move_to_pose(position=[0.5, 0.0, 0.25], quat_xyzw=quat_xyzw_node1, cartesian=cartesian)
        moveit2_2.move_to_pose(position=[0.5, 0.0, 0.25], quat_xyzw=quat_xyzw_node2, cartesian=cartesian)
        moveit2_1.wait_until_executed()
        moveit2_2.wait_until_executed()
        print("rest")


    except Exception as err:
        node1.get_logger().info(f'Exception occured. {err}')
        node2.get_logger().info(f'Exception occured. {err}')


    node1.get_logger().info(f'Movement completed')
    node2.get_logger().info(f'Movement completed')
    rclpy.shutdown()
    exit(0)

'''
def reset_collision_objects(node):
    # Create a PlanningScene message
    planning_scene = PlanningScene()

    # Set the operation to REMOVE or CLEAR to remove or clear all collision objects respectively
    planning_scene.world.collision_objects.clear()
    planning_scene.is_diff = True

    # Publish the PlanningScene to update the collision objects in the planning scene
    publisher = node.create_publisher(PlanningScene, "planning_scene", 10)
    publisher.publish(planning_scene)

    # Wait for the update to be applied
    apply_scene_service = node.create_client(ApplyPlanningScene, "apply_planning_scene")
    while not apply_scene_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the /apply_planning_scene service...")
    request = ApplyPlanningScene.Request()
    #apply_scene_service.call(request)
'''


def add_ground_plane(node1):

    # Create a CollisionObject message
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box shape
    ground_plane = Plane()
    ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

    # Set the ground plane's pose
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

    collision_object.planes.append(ground_plane)
    collision_object.plane_poses.append(ground_plane_pose)

    # Create a PlanningScene message
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True
    
    publisher_ = node1.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

def add_ground_plane(node2):

    # Create a CollisionObject message
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box shape
    ground_plane = Plane()
    ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

    # Set the ground plane's pose
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

    collision_object.planes.append(ground_plane)
    collision_object.plane_poses.append(ground_plane_pose)

    # Create a PlanningScene message
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True
    
    publisher_ = node2.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

if __name__ == "__main__":
    main()
