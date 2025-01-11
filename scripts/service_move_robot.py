#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningFeedback
from assignment2_rt_part1.srv import get_last_target, get_last_targetResponse

def handle_set_target(req):
    target_x = req.x
    target_y = req.y

    rospy.loginfo(f"Recived coordinate: x={target_x}, y={target_y}")
 
    # Create an action client to communicate with the action server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    rospy.loginfo("Connected to action server")
    goal = PlanningGoal()  # Create a goal instance
    goal.target_pose.pose.position.x = target_x  # Set the target x-coordinate
    goal.target_pose.pose.position.y = target_y  # Set the target y-coordinate
    client.send_goal(goal)

    client.wait_for_result()
    result_state = client.get_state()

    if result_state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached")
    else:
        rospy.loginfo("failed")

def set_target_service_node():

    rospy.init_node('set_target_service_node')
    service = rospy.Service('get_last_target', get_last_target, handle_set_target)
    rospy.loginfo("service started")
    rospy.spin()

if __name__ == '__main__':
    try:
        set_target_service_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")