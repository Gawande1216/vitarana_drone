#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import rospy
import time
import tf
import time


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # Declaring all the variables
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.prev_error = [0,0,0]
        self.error_sum = [0,0,0]
        self.error = [0,0,0]
        self.set_point = [19.0,72.0, 3]
        self.error_difference = [0,0,0]

        self.a=False
        self.b=False

        self.sample_time = 30

        #Kp = [0,0] Kp[0] corresponds to Kp for latitude, Kp[1] for longitude and so for for Ki and Kd
        self.Kp = [3.2,10]
        self.Ki = [0.,0]
        self.Kd = [5.3,15]

        # All the Publishers
        # self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_error', Float64, queue_size=1)
        self.latitude_error_pub = rospy.Publisher('/latitude_error', Float64, queue_size=1)
        self.longitude_error_pub = rospy.Publisher('/longitude_error', Float64, queue_size=1)
        self.latitude_pid_pub = rospy.Publisher('/latitude_pid', Float64, queue_size=1)
        self.longitude_pid_pub = rospy.Publisher('/longitude_pid', Float64, queue_size=1)
        self.z_error = rospy.Publisher('/z_error', Float64, queue_size=1)
        self.zero_error = rospy.Publisher('/zero_error', Float64, queue_size=1)
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # All the Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch',  PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw',  PidTune, self.yaw_set_pid)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        # rospy.Subscriber('/set_latitude', Float64, self.set_latitude)
        
        self.edrone_cmd_object = edrone_cmd()
        self.edrone_cmd_object.rcRoll = 0
        self.edrone_cmd_object.rcPitch = 0
        self.edrone_cmd_object.rcThrottle = 0

    def gps_callback(self, msg):
       	self.latitude = msg.latitude
       	self.longitude = msg.longitude
       	self.altitude = msg.altitude

    def altitude_set_pid(self, msg):
    	self.Kp[1] = msg.Kp * 0.5
    	self.Ki[1] = msg.Ki * 0.01
    	self.Kd[1] = msg.Kd * 0.5
    
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.5  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    def pitch_set_pid(self, pitch):
    	self.Kp[1] = pitch.Kp * 0.5
    	self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd

    # def yaw_set_pid(self, yaw):
    # 	self.Kp[2] = yaw.Kp * 0.5
    # 	self.Ki[2] = yaw.Ki * 0.008
    #     self.Kd[2] = yaw.Kd 

    # def set_latitude(self, msg):
    #     self.set_point[0] = msg.data
    #     print(self.set_point)
	

    def pid(self):
        # print(round(self.altitude, 5))


        
        # This is altitude error which is published on attitude controller
        self.error[2] = self.set_point[2] - self.altitude
        

        #Index 0 for latitude and 1 for longitude and 2 for altitude
        self.error[0] = self.set_point[0] - self.latitude
        self.error_sum[0]+=self.error[0] * 0.033
        self.error_difference[0] = (self.error[0] - self.prev_error[0])/0.033
        output_latitude = self.Kp[0] * self.error[0] + self.Ki[0] * self.error_sum[0] + self.Kd[0] * self.error_difference[0]
        self.prev_error[0] = self.error[0]

        # Index 1 for longitude
        self.error[1] = self.set_point[1] - self.longitude
        self.error_sum[1]+=self.error[1] * 0.033
        self.error_difference[1] = (self.error[1] - self.prev_error[1])/0.033
        output_longitude = self.Kp[1] * self.error[1] + self.Ki[1] * self.error_sum[1] + self.Kd[1] * self.error_difference[1]
        self.prev_error[1] = self.error[1]
        
        self.edrone_cmd_object.rcRoll = self.error[0]
        self.edrone_cmd_object.rcPitch = self.error[1]
        self.edrone_cmd_object.rcThrottle = self.error[2]

        # Publishing to edrone_cmd topic
        self.drone_cmd_pub.publish(self.edrone_cmd_object)
        # self.drone_cmd_pub.publish(self.drone_cmd_pub.rcPitch)


        # self.longitude_error_pub.publish(self.error[1])
        self.latitude_error_pub.publish(self.error[0])
        self.latitude_pid_pub.publish(output_latitude)
        # self.longitude_error_pub.publish(output_longitude)
        # print(output_latitude)
        # self.throttle_pub.publish(self.error[2])
        # self.zero_error.publish(1500)

        # print(self.latitude)
        if(self.a==False and round(self.altitude, 2) == 3.05):
            self.set_point[0] = 19.0000451704 
            self.Kp[0] = 665.5
            self.Ki[0] = 0
            self.Kd[0] = 6000
            self.a = True
            print("Set Point Changed")
        
        if(self.a==True and self.b == False and 19.0000406534 <self.latitude < 19.0000451704):
            # time.sleep(1.2) 434905 412765 434905
            self.set_point[2] = 0.0
            self.set_point[0] = 19.0000451704
            self.set_point[1] = 72.0
            self.Kp[0] = 3
            self.Kd[0] = 5.3
            print("Set Point Chnaged")
            self.b=True
        # print(self.altitude)
        # if(a==True and b==True and round(self.altitude, 3) == 0.366):
        #     self.zero_error.publish(0)
        #     print("ccccccccccccccccccccccccccccccccccccccccccccccccc")





if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()