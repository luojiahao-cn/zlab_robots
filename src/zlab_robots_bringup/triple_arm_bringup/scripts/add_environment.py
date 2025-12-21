#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import sys

def add_environment():
    rospy.init_node('add_environment_node')
    
    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    
    rospy.sleep(2) # Wait for scene to initialize

    # Add a table
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = "world"
    p.pose.position.x = 0.8
    p.pose.position.y = 0.0
    p.pose.position.z = 0.4
    p.pose.orientation.w = 1.0
    
    # Table dimensions (x, y, z)
    scene.add_box("table", p, (0.5, 1.5, 0.8))
    
    rospy.loginfo("Added table to the planning scene")

if __name__ == '__main__':
    try:
        add_environment()
    except rospy.ROSInterruptException:
        pass