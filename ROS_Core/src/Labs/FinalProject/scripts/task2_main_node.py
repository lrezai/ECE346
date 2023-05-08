#!/usr/bin/env python
import rospy
import sys, os
from task2_world.task2_main import TaskTwo

if __name__ == '__main__':
    # Safe guard for GPU memory
    rospy.init_node('task2_main_node.py')
    rospy.loginfo("Task 2 Main node")
    TaskTwo()
    rospy.spin()