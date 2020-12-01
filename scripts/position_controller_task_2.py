#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import rospy
import time
import tf
import time
import rospy
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String


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
        self.set_point = [19.0009248718,71.9998318945,23]
        self.error_difference = [0,0,0]
        self.set_point_d = [0,0,0]
        self.range_finder = [0,0,0,0,0]

        self.a=False
        self.b=False
        self.c = False
        self.d = False

        self.sample_time = 30

        #Kp = [0,0] Kp[0] corresponds to Kp for latitude, Kp[1] for longitude and so for for Ki and Kd
        self.Kp = [3,100]
        self.Ki = [0,200]
        self.Kd = [5.3,15]

        # /////////////////////////////////////////
        self.service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        self.service.wait_for_service()

        self.gripper_check = "False"
        # gripper_service = rospy.Service('/edrone/activate_gripper', Gripper, self.gripper_service_server_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        # print("c")
        self.isTrue = False
# /////////////////////////////////////////////////////

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
        rospy.Subscriber('/qr_latitude', Float64, self.qr_latitude_callback)
        rospy.Subscriber('/qr_longitude', Float64, self.qr_longitude_callback)
        rospy.Subscriber('/qr_altitude', Float64, self.qr_altitude_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        
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
    	self.Ki[1] = pitch.Ki * 0.5
        self.Kd[1] = pitch.Kd * 0.5

    def qr_latitude_callback(self, msg):
        self.set_point_d[0] = 19
        # print(msg.data)

    def qr_longitude_callback(self, msg):
        self.set_point_d[1] = 72
        # print(msg.data)

    def qr_altitude_callback(self, msg):
        self.set_point_d[2] = 25
        # print(msg.data)


    def function_repeat(self):
        if(self.gripper_check == "True" and self.isTrue == False):
            service_response = self.service(True)
            if(service_response.result == True):
                self.isTrue = True
                print(str(service_response.result) + " Parcel picked up")
    # else:
    #     service_response = self.service(False)
    #     print(str(service_response.result) +  " Parcel not picked up")

    def gripper_check_callback(self, msg):
        # print(msg.data)
        self.gripper_check = msg.data
    # print(msg.data)

    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def range_finder_top_callback(self, msg):
        self.range_finder = msg.ranges

    # if you use this for control, you may have to change the relevant pitch   direction because of the sign


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
        if(self.a==False and self.error[2] < 0.1 and self.error[2] > -0.1):
            self.set_point[0] = 19.0007046575 + 0.00000268549 + 9.87099998e-7
            self.set_point[1] = 71.9998955286 - 0.0000022222222
            # self.Kp[0] = 665.5
            # self.Ki[0] = 0
            # self.Kd[0] = 6000
            self.a = True
            print("Set Point Changed")
        # print(self.altitude)
        # print(self.latitude)
        # print(self.longitude)
        self.function_repeat()  
        if(self.a==True and self.b == False and -0.000004517 < self.error[0] < 0 and -0.000004517 < self.error[1] < 0):
            # time.sleep(1.2) 434905 412765 434905 -0.00000269145 71.9998955764 19.000699838 71.999896074 19.000701972 71.9998954694 19.0007029591 71.9998953407 19.0007053134 71.9998953544


            # print(self.error[0])
            self.set_point[2] = 21
            # print(self.error)
            # print(self.latitude)
            # print(self.longitude)
            # self.set_point[0] = 19.0000451704
            # self.set_point[1] = 72.0
            # self.Kp[0] = 3
            # self.Kd[0] = 5.3
            print("Set Point Changed")
            self.b=True
        
        if(self.b and self.c == False and self.altitude < 22.2 and self.isTrue):
            # self.set_point = self.set_point_d
            self.set_point[2] = 25
            # print(self.set_point)
            # time.sleep(1)
            self.c = True

        if(self.b and self.c and self.altitude >= 25):
            self.set_point[0] = 19.0
            self.set_point[1] = 72.0
            # time.sleep(1.2)

        if(self.range_finder[3] > 0.3 and self.range_finder[3] < 0.5 and self.isTrue and self.d == False):
            print(self.range_finder)
            self.set_point[0] = self.latitude
            self.set_point[1] = self.longitude
            self.d == True
            # print(self.set_point, "Obstacle Therefore CHnaged SetPoint")

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