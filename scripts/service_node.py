#!/usr/bin/env python

import rospy
from assignment2_rt_part1.srv import get_last_target, get_last_targetResponse
from assignment_2_2024.msg import PlanningActionGoal
from geometry_msgs.msg import PoseStamped

# Global variable to store last target coordinates
last_target_x = 0.0
last_target_y = 0.0

def planning_callback(msg):
    """Callback function to update the last target coordinates from the PlanningActionGoal."""
    global last_target_x, last_target_y
    # Access the target pose from the goal and update the coordinates
    last_target_x = msg.goal.target_pose.pose.position.x
    last_target_y = msg.goal.target_pose.pose.position.y

def handle_get_last_target(req):
    """Service callback to return the last target coordinates."""
    return get_last_targetResponse(last_target_x, last_target_y)

def service_node():
    """Service node to provide the last target coordinates."""
    rospy.init_node('service_node')

    # Subscribe to the /reaching_goal/goal topic (PlanningActionGoal)
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, planning_callback)

    # Define the service with the name 'get_last_target' and the handler
    rospy.Service('get_last_target', get_last_target, handle_get_last_target)

    rospy.loginfo("Service 'get_last_target' is ready")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        service_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target service node terminated.")