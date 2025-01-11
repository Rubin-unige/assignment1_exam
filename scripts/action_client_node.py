#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningFeedback
from nav_msgs.msg import Odometry
from assignment2_rt_part1.msg import robot_status
from assignment2_rt_part1.srv import get_last_target, get_last_targetResponse
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32 

# Global variables to store the robot's position and velocity
current_x = 0.0
current_y = 0.0
vel_x = 0.0
vel_z = 0.0
cancel_goal_flag = False  # Flag to cancel the goal
goal_active = False  # Indicates if a goal is currently being pursued
closest_distance = float('inf')

def odom_callback(msg):
    """Callback function to update robot's position and velocity from /odom."""
    global current_x, current_y, vel_x, vel_z
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_z = msg.twist.twist.angular.z

def feedback_callback(feedback):
    if feedback.stat == "Target reached!":
        rospy.loginfo("Goal Reached!! Press 'Enter' to continue!!")
 
def send_goal(client, target_x, target_y):
    """Send the goal to the action server."""
    global goal_active
    goal = PlanningGoal()  # Create a goal instance
    goal.target_pose.pose.position.x = target_x  # Set the target x-coordinate
    goal.target_pose.pose.position.y = target_y  # Set the target y-coordinate
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    rospy.loginfo(f"Sending goal: x={target_x}, y={target_y}")
   
    # Send the goal to the action server and listen for feedback
    client.send_goal(goal, feedback_cb=feedback_callback)
    goal_active = True  # Mark the goal as active

def cancel_goal(client):
    """Cancel the goal if the cancel flag is set and the robot is moving."""
    global cancel_goal_flag, goal_active
    if cancel_goal_flag and goal_active:
        client.cancel_goal()

def get_last_target_service():
    try:
        rospy.wait_for_service('get_last_target')
        get_last_target_client = rospy.ServiceProxy('get_last_target', get_last_target)
        response = get_last_target_client()
        rospy.loginfo(f"Last Target: x={response.x}, y = {response.y}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def laser_callback(msg):
    global closest_distance
    laser_ranges = msg.ranges
    closest_distance = min(laser_ranges)

def action_client_node():
    """Main action client node."""
    global cancel_goal_flag, goal_active
    
    rospy.init_node('action_client_node')
    
    # Create an action client to communicate with the action server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    rospy.loginfo("Connected to action server")

    # Subscribe to /odom to get robot's position and velocity
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Publisher to send robot's position and velocity to the topic
    pub = rospy.Publisher('/robot_status', robot_status, queue_size=10)

    obs_pub = rospy.Publisher('/obs_distance', Float32, queue_size = 10)
    
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter 'goal' to send goal, and 'last' to get last coordinate: ").strip().lower()

            if user_input == 'goal':
                # Get user input for target coordinates
                target_x = float(input("Enter target x coordinate: "))
                target_y = float(input("Enter target y coordinate: "))

                # Send the goal to the action server
                send_goal(client, target_x, target_y)
                obs_pub.publish(closest_distance)

                # Monitor the goal state and robot's position while the goal is in progress
                while not rospy.is_shutdown() and client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                    # Check for user input to cancel the goal
                    user_input = input("Robot running!! Enter 'cancel' to cancel the current goal: ").strip().lower()
                    if user_input == 'cancel':
                        cancel_goal_flag = True
                        cancel_goal(client)
                        break
                        
                    # Publish robot's current position and velocity
                    pos_vel_msg = robot_status()
                    pos_vel_msg.x = current_x
                    pos_vel_msg.y = current_y
                    pos_vel_msg.vel_x = vel_x
                    pos_vel_msg.vel_z = vel_z
                    pub.publish(pos_vel_msg)

                    rospy.sleep(0.5)  # Sleep for 500 ms to avoid spamming messages

                # Check the outcome of the goal
                if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Please Enter new coordinates !!")
                elif cancel_goal_flag:
                    rospy.loginfo("Goal was canceled by the user.")
                else:
                    rospy.loginfo("Goal did not succeed. Status: %d", client.get_state())

                cancel_goal_flag = False
                goal_active = False  # Reset goal_active after processing the goal
            
            elif user_input == 'last':
                get_last_target_service()
            
            else:
                rospy.loginfo("Invalid Input !!")

        except ValueError:
            rospy.logerr("Invalid input! Please enter valid numbers for the target coordinates.")
        
        # After handling the goal or cancelation, return to accepting new coordinates
        cancel_goal_flag = False  # Reset the cancel flag before the next iteration
        rospy.sleep(0.5)  # Sleep for 1 second before allowing another input


if __name__ == '__main__':
    try:
        action_client_node()  # Run the Action Client
    except rospy.ROSInterruptException:
        rospy.loginfo("Action client node terminated.")
