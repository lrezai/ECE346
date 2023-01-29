import rospy
from .ros_utility import get_ros_param
import numpy as np

class GeneratePwm():
    def __init__(self):
        
        self.d_open_loop = np.array([-0.0146, 0.0874, -0.0507, 0.0005, -0.0332, 0.1869, 0.0095, 0.0170, -0.0583, 0.0388])
        self.read_parameters()
        
    def read_parameters(self):
        self.max_throttle = get_ros_param('~max_throttle', 0.2)
        self.min_throttle = get_ros_param('~min_throttle', -0.3)
        
        
    def convert(self, accel, steer, state):
        
        v = state.v_long        
        if accel<0:
            throttle_pwm = accel/10 - 0.5
        else:
            temp = np.array([v**3, v**2, v, accel**3, accel**2, accel, v**2*accel, v*accel**2, v*accel, 1])
            d = temp@self.d_open_loop
            d += min(steer*steer*0.5,0.05)
        
        throttle_pwm = np.clip(d, self.min_throttle, self.max_throttle)
        steer_pwm = np.clip(steer/0.3, -1, 1)
        
        return throttle_pwm, steer_pwm
        
        
        