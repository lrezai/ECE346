#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from racecar_routing.srv import Plan, PlanResponse, PlanRequest
from nav_msgs.msg import Path as PathMsg # used to display the trajectory on RVIZ
from nav_msgs.msg import Odometry


def get_ros_param(param_name: str, default):
    '''
    Read a parameter from the ROS parameter server. If the parameter does not exist, return the default value.
    Args:
        param_name: string, name of the parameter
        default: default value
    Return:
        value of the parameter
    '''
    if rospy.has_param(param_name):
        return rospy.get_param(param_name)
    else:
        # try seach parameter
        if param_name[0] == '~':
            search_param_name = rospy.search_param(param_name[1:])
        else:
            search_param_name = rospy.search_param(param_name)

        if search_param_name is not None:
            rospy.loginfo('Parameter %s not found, search found %s, using it', param_name, search_param_name)
            return rospy.get_param(search_param_name)
        else:
            rospy.logwarn("Parameter '%s' not found, using default: %s", param_name, default)
            return default

class BroadWay():
    def __init__(self):
        
        # Read ROS topic names to subscribe 
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        # Subscribe to odom_topic
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)
    
        self.path_pub = rospy.Publisher('Routing/Path', PathMsg, queue_size=10)
        
        rospy.wait_for_service('/routing/plan')
        
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)
        
        
    def odometry_callback(self, odom_msg):
        '''
        Subscriber callback function of the robot pose
        '''
        # Add the current state to the buffer
        # Controller thread will read from the buffer
        # Then it will be processed and add to the planner buffer 
        # inside the controller thread
        self.odom_msg = odom_msg
        
        
    def pub_response(self):
        
        with open('task1.yaml', 'r') as file:
            load_pts = yaml.load(file, loader = yaml.Loader)
        '''
        load_pts = {
            goal_1: [3.15, 0.15],
            goal_2: [3.15, 0.47]
            goal_3: [5.9, 3.5]
            goal_4: [5.6, 3.5]
            goal_5: [0.15, 3.5]
            goal_6: [0.45, 3.5]
            goal_7: [3, 1.1]
            goal_8: [3, 0.8]
            goal_9: [3, 2.2]
            goal_10: [0.75,2.1]
            goal_11: [0.75,4.3]
            goal_12: [4.6,4.6]
            goal_order: [1, 3, 9, 8, 10, 11, 12, 6, 2, 4, 9, 7, 9, 12, 5, 1]
            ETC
        }
        '''
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size = 1)



        NUM_TRANSITIONS = 15
        goal_order = load_pts['goal_order']
        load_pts_vals = load_pts.values()
        '''
        load_pts = [
            goal_1: [3.15, 0.15],
            goal_2: [3.15, 0.47]
            goal_3: [5.9, 3.5]
            goal_4: [5.6, 3.5]
            goal_5: [0.15, 3.5]
            goal_6: [0.45, 3.5]
            goal_7: [3, 1.1]
            goal_8: [3, 0.8]
            goal_9: [3, 2.2]
            goal_10: [0.75,2.1]
            goal_11: [0.75,4.3]
            goal_12: [4.6,4.6]
            goal_order: [1, 3, 9, 8, 10, 11, 12, 6, 2, 4, 9, 7, 9, 12, 5, 1]
            ETC
        ]
        '''

        num_times = 0

        while num_times < NUM_TRANSITIONS:
            
            
            
            x_start = load_pts_vals[goal_order[num_times]-1][0]# 0 x coordinate of the start
            y_start = load_pts_vals[goal_order[num_times]-1][1]# y coordinate of the start

            x_goal =  load_pts_vals[goal_order[num_times+1]-1][0] # x coordinate of the goal
            y_goal =  load_pts_vals[goal_order[num_times+1]-1][1]# y coordinate of the goal

            plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
            plan_response = self.plan_client(plan_request)
        
            path_msg = plan_response.path
            path_msg.header.stamp = rospy.get_rostime()
            path_msg.header.frame_id = 'map'

        
            if np.linalg.norm([self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y],[x_goal,y_goal]) < 0.7:
                self.path_pub.publish(path_msg)

            num_times += 1 
    
if __name__ == '__main__':
    rospy.init_node('broadcast_waypoint_node')
    rospy.loginfo("Broadcast Waypoint node")
    BroadWay()
    rospy.spin()
    
    
    # call ilqr with cost function (tracking) + getting close to obstacle
    # define cost function ignoring far away peaks for this planning cycle to only include obstacles within horizon
    # minimize this cost function
    # add into trajectory planner node