#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math
from time import sleep

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def get_distance(pose1, pose2):
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.sqrt(dx**2 + dy**2)

def main():
    # Initialize ROS2 communications and Simple Commander API
    rclpy.init()
    nav = BasicNavigator()

    # Set initial pose (comment this out if the initial pose is already set)
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    #nav.setInitialPose(initial_pose)

    # Wait for Nav2 to become active
    nav.waitUntilNav2Active()

    # Create some Nav2 goal poses
    goal_pose1 = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
    goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, 0.0)

    # Define waypoints
    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    general_distance_threshold = 0.5  # General distance threshold in meters
    special_distance_threshold = 0.3  # Smaller distance threshold for the special waypoint
    wait_at_waypoint_index = 1  # Index of the waypoint where the robot should stop and wait

    for i, waypoint in enumerate(waypoints):
        nav.goToPose(waypoint)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                current_pose = feedback.current_pose.pose
                if i == wait_at_waypoint_index:
                    distance_to_waypoint = get_distance(current_pose, waypoint.pose)
                    if distance_to_waypoint < special_distance_threshold:
                        nav.cancelTask()
                        print(f'Reached within {special_distance_threshold} meters of waypoint {i+1}.')
                        print('Waiting for 10 seconds at this waypoint.')
                        sleep(10)
                        pass
                else:
                    distance_to_waypoint = get_distance(current_pose, waypoint.pose)
                    if distance_to_waypoint < general_distance_threshold:
                        nav.cancelTask()
                        print(f'Reached within {general_distance_threshold} meters of waypoint {i+1}.')
                        pass

            sleep(0.1)  # Small delay to avoid busy-waiting

    # Get the result of the last waypoint navigation
    print(nav.getResult())

    # Shutdown ROS2 communications
    rclpy.shutdown()

if __name__ == '__main__':
    main()
