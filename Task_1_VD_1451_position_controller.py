#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
import rospy
import time
import tf

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller') 

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.eDrone_cmd = edrone_cmd()

        self.sample_time = 30

        self.current_drone_position = [0, 0, 0]
        self.drone_setpoint = [ 19.0000000000, 72.0000000000, 15.0]
        
        #[latitude, longitude]
        self.Kp = [3, 8]
        self.Ki = [0, 0]
        self.Kd = [5.3, 7]

        #[latitude, longitude]
        self.error = [0, 0]
        self.prev_error = [0, 0]
        self.error_summation = [0, 0]
        self.dErr = [0, 0]
        
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd , queue_size=1)
        
        # For changing setopints of drone
        self.flag1 = False
        self.flag2 = False

    def gps_callback(self,gps):
        self.current_drone_position[0] = gps.latitude
        self.current_drone_position[1] = gps.longitude
        self.current_drone_position[2] = gps.altitude

    # Function for calculating and publishing errors to attitude_controller.py
    def pid(self):
        # Calculating errors in latitude, longitude and altitude
        self.error[0] = self.drone_setpoint[0] - self.current_drone_position[0]
        self.error[1] = self.drone_setpoint[1] - self.current_drone_position[1]
        self.eDrone_cmd.rcThrottle =  self.drone_setpoint[2] - self.current_drone_position[2]
        
        # Our sample time is 30 therefore we are using time_interval = 0.033 i.e. 1/30
        time_interval = 0.033

        self.error_summation[0] += self.error[0]*time_interval
        self.error_summation[1] += self.error[1]*time_interval

        self.dErr[0] = (self.error[0]-self.prev_error[0])/time_interval
        self.dErr[1] = (self.error[1]-self.prev_error[1])/time_interval
   
        # Calculating total roll and pitch values to be published to attitude_controller.py
        self.eDrone_cmd.rcRoll = self.Kp[0]*self.error[0] + self.Ki[0]*self.error_summation[0] + self.Kd[0]*self.dErr[0]
        self.eDrone_cmd.rcPitch = self.Kp[1]*self.error[1] + self.Ki[1]*self.error_summation[1] + self.Kd[1]*self.dErr[1]

        # Conditions to change set points of drone
        # Condition 1: When Drone reaches altitude of 3m, its latitude setpoint is changed (to keep it more precise we have given range of 0.1m from current setpoint)
        # Condition 2: When drone reaches second setopint i.e. latitude == 19.0000451704, its final setpoint is changed 
        #              (to keep it more precise we have given lesser range than the given tolerance i.e. +-0.000004517)
        # Condition 3: When drone reaches final setpoint we are logging message DONE !

        if(not self.flag1  and self.eDrone_cmd.rcThrottle<0.1 and self.eDrone_cmd.rcThrottle>-0.1):
            print("Change")
            self.drone_setpoint[0] = 19.0000451704
            # self.drone_setpoint[1] = 71.9998955286
            self.flag1 = True

        # if(self.flag1 and not self.flag2 and -0.000040517 < self.error[0] < 0):
        #     self.drone_setpoint[2] = 0.31
        #     self.flag2 = True

        if(self.flag2 and self.current_drone_position[2] == 0.31):
            rospy.loginfo("DONE")

        # Publishing calculated errors to attitude_controller.py
        self.drone_cmd_pub.publish(self.eDrone_cmd)

        # Updating previous errors for PID
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        
if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
