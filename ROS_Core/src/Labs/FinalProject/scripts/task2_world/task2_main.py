import rospy
import numpy as np
from .util import get_ros_param, get_text_msg, RefPath, extract_state_odom
from .boss import BossPlanner
from racecar_routing.srv import Plan, PlanRequest
from final_project.srv import Task, TaskResponse, TaskRequest, Reward, RewardResponse, RewardRequest, Schedule, ScheduleResponse, ScheduleRequest
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from visualization_msgs.msg import Marker, MarkerArray
import threading
import yaml
from typing import List, Tuple
import time

class TaskTwo:
    def __init__(self) -> None:
        
        # initalize positions for callbacks
        self.truck_x = None
        self.truck_y = None
        self.boss_x = None
        self.boss_y = None
        
        # stores recent positions of pose
        self.boss_rec_pose = []
        self.boss_safe_dist = 0.4
        self.boss_safe_x = 0
        self.boss_safe_y = 0
        
        # Read ROS topic names to subscribe 
        self.odom_topic = get_ros_param('~odom_topic', '/Simulation/Pose')
        self.odom_msg = None
        
        # initalize subscribers to positions
        self.truck_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)
        self.boss_sub = rospy.Subscriber('/Boss/Pose', Odometry, self.boss_callback, queue_size=10)
        
        # initialize warehouse information
        self.setup_warehouse()
        
        # initialize path
        self.path_pub = rospy.Publisher('Routing/Path', PathMsg, queue_size=10)
        
        # set up the service clients
        self.setup_clients()
        
        threading.Thread(target=self.loop).start()

    def odometry_callback(self, odom_msg):
        self.truck_x = odom_msg.pose.pose.position.x
        self.truck_y = odom_msg.pose.pose.position.y
        
    def boss_callback(self, odom_msg):
        self.boss_x = odom_msg.pose.pose.position.x
        self.boss_y = odom_msg.pose.pose.position.y
   
        self.boss_rec_pose.insert(0, [self.boss_x, self.boss_y])
        
        counter = 1
        
        while (counter < len(self.boss_rec_pose)):
            
            point = self.boss_rec_pose[counter*-1]
            dist_to_boss = np.linalg.norm([self.boss_x - point[0], self.boss_y - point[1]])
            
            if dist_to_boss < self.boss_safe_dist:
                break
            
            counter += 1
            
            remove = counter - 2
            
            while remove > 0:
                self.boss_rec_pose.pop()
                remove -= 1
                
            if counter != 1:
                self.boss_safe_x = self.boss_rec_pose[-1][0]
                self.boss_safe_y = self.boss_rec_pose[-1][1]
            
    def setup_warehouse(self):
        # Load parameters
        with open('/Users/lil/ECE346_FP/ECE346/ROS_Core/src/Labs/FinalProject/task2.yaml', "r") as stream:
            self.warehouse_info = yaml.safe_load(stream)
            
        # HACK: No failsafe is implemented for invalid warehouse config
        self.warehouse_location = []
        for warehouse in self.warehouse_info.values():
            location_info = warehouse['location']
            location_info.extend(warehouse['dxdy'])
            self.warehouse_location.append(location_info)
        self.num_warehouse = len(self.warehouse_location)
               
    def setup_clients(self):
        rospy.wait_for_service('/SwiftHaul/Start')
        self.client_start = rospy.ServiceProxy('/SwiftHaul/Start', Empty)
        rospy.wait_for_service('/SwiftHaul/BossSchedule')
        self.client_get_boss_schedule = rospy.ServiceProxy('/SwiftHaul/BossSchedule', Schedule) 
        rospy.wait_for_service('/SwiftHaul/SideTask')
        self.client_get_side_task = rospy.ServiceProxy('/SwiftHaul/SideTask', Task)
        rospy.wait_for_service('/SwiftHaul/BossTask')
        self.client_get_boss_task = rospy.ServiceProxy('/SwiftHaul/BossTask', Task)
        rospy.wait_for_service('/SwiftHaul/GetReward')
        self.client_get_reward = rospy.ServiceProxy('/SwiftHaul/GetReward', Reward)
        rospy.wait_for_service('/routing/plan')
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)  
        
              
    def loop(self):
        
        rospy.sleep(1)
        warehouse_idx = None
        reward_response = 0
        
        while self.boss_safe_x == self.boss_safe_y:
            rospy.sleep(1)

        while rospy.is_shutdown() is False:
            
            if self.truck_x is None or self.boss_x is None:
                continue
            
            if warehouse_idx is None or warehouse_idx == -1:
                warehouse_idx = self.client_get_boss_task(TaskRequest()).task

            warehouse_x = self.warehouse_location[warehouse_idx][0]
            warehouse_y = self.warehouse_location[warehouse_idx][1]
            dx = self.warehouse_location[warehouse_idx][2]
            dy = self.warehouse_location[warehouse_idx][3]
            
            
            while rospy.is_shutdown() is False and (np.abs(self.truck_x - warehouse_x) > dx or np.abs(self.truck_y - warehouse_y) > dy):
            
                # publish info
                plan_request = PlanRequest([self.truck_x, self.truck_y], [self.boss_safe_x, self.boss_safe_y])
                plan_response = self.plan_client(plan_request)
                path_msg = plan_response.path
                
                path_msg.header.stamp = rospy.get_rostime()
                path_msg.header.frame_id = 'map'
                self.path_pub.publish(path_msg)
                
                time.sleep(1)
            
            
            if np.abs(self.truck_x - warehouse_x) < dx and np.abs(self.truck_y - warehouse_y) < dy:
            
                reward_response = self.client_get_reward(RewardRequest(warehouse_idx)).total_reward
                
                while(self.client_get_boss_task(TaskRequest()).task == -1):
                    rospy.sleep(0.1)
                    
                rospy.sleep(0.1)
                warehouse_idx = None
    