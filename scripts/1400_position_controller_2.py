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
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # Declaring all the variables
        self.drone_position = [0.0, 0.0, 0.0]
        self.setpoint = [19.0009248718,71.9998318945, 25]
        self.altitude = 0.0
        self.altitude_setpoint = 0.0
        self.roll = 0
        self.pitch = 0
        self.throttle = 0
        # self.latitude = 0
        # self.longitude = 0
        # self.altitude = 0
        self.prev_error = [0,0,0]
        self.error_sum = [0,0,0]
        self.error = [0,0,0]
        self.d_error = [0,0,0]
        self.range_finder_top = [0,0,0,0,0]
        self.qr_set_point = [0,0,0]
        self.qr_check = False
        self.setpoint_original = [0,0,0]
        self.isDrop = False
        # self.set_point = [19.0009248718,71.9998955286, 25]
        # self.error_difference = [0,0,0]

        self.a = False
        self.b = False
        self.c = False
        self.d = False
        self.e = False
        self.f = False
        self.g = False
        self.h = False
        self.i = False
        self.k = False
        self.l = False
        self.count = 1


        self.sample_time = 0.016

        #Kp = [0,0] Kp[0] corresponds to Kp for latitude, Kp[1] for longitude and so for for Ki and Kd
        self.Kp = [0.44,0.4,44]
        self.Ki = [0.0,0.0,0.0]
        self.Kd = [50.37,50.62,5000]
        # All the Publishers
        # self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        # self.throttle_pub = rospy.Publisher('/throttle_error', Float64, queue_size=1)
        # self.latitude_error_pub = rospy.Publisher('/latitude_error', Float64, queue_size=1)
        # self.longitude_error_pub = rospy.Publisher('/longitude_error', Float64, queue_size=1)
        # self.latitude_pid_pub = rospy.Publisher('/latitude_pid', Float64, queue_size=1)
        # self.longitude_pid_pub = rospy.Publisher('/longitude_pid', Float64, queue_size=1)
        # self.z_error = rospy.Publisher('/z_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('y_error', Float32, queue_size=1)
        self.throttle_error_pub = rospy.Publisher('z_error', Float32, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # All the Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        rospy.Subscriber('/qr_latitude', String, self.qr_latitude_callback)
        rospy.Subscriber('/qr_longitude', String, self.qr_longitude_callback)
        rospy.Subscriber('/qr_altitude', String, self.qr_altitude_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/qr_check', String, self.qr_check_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch',  PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw',  PidTune, self.yaw_set_pid)
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        # rospy.Subscriber('/set_latitude', Float64, self.set_latitude)
        
        self.edrone_cmd_object = edrone_cmd()
        self.edrone_cmd_object.rcRoll = 0
        self.edrone_cmd_object.rcPitch = 0
        self.edrone_cmd_object.rcYaw = 0
        self.edrone_cmd_object.rcThrottle = 0

        self.service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        self.service.wait_for_service()
        self.gripper_check = "False"
        self.isTrue = False

    def gps_callback(self, msg):
       	self.drone_position[0] = msg.latitude
       	self.drone_position[1] = msg.longitude
       	self.drone_position[2] = msg.altitude
        # print(self.drone_position[2])
    
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.01  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.01

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    def pitch_set_pid(self, pitch):
    	self.Kp[1] = pitch.Kp * 0.01
    	self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.1

    def altitude_set_pid(self, msg):
    	self.Kp[2] = msg.Kp
    	self.Ki[2] = msg.Ki * 0.01
    	self.Kd[2] = msg.Kd

    def function_repeat(self, request):
        if(self.gripper_check == "True" and self.isTrue == False):
            service_response = self.service(True)
            if(service_response.result == True):
                self.isTrue = True
                print(str(service_response.result) + " Parcel picked up")
        if(request == False):
            service_response = self.service(False)
            if(service_response.result == False):
                print(str(service_response.result) + " Parcel Droped")
                self.isDrop = True


    def gripper_check_callback(self, msg):
        self.gripper_check = msg.data

    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def range_finder_top_callback(self, msg):
        self.range_finder_top = msg.ranges

    def qr_latitude_callback(self, msg):
        self.qr_set_point[0] = float(msg.data)

    def qr_longitude_callback(self, msg):
        self.qr_set_point[1] = float(msg.data)

    def qr_altitude_callback(self, msg):
        self.qr_set_point[2] = float(msg.data)

    def qr_check_callback(self, msg):
        if(msg.data == "True"):
            self.qr_check = True

    def pid(self):
        # print(self.drone_position[2])
        self.error[0] = (self.setpoint[0] - self.drone_position[0])*1000000
        self.error[1] = (self.setpoint[1] - self.drone_position[1])*1000000
        self.error[2] = (self.setpoint[2] - self.drone_position[2])

        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_sum[2] = self.error_sum[2] + self.error[2]

        self.d_error[0] = self.error[0] - self.prev_error[0]
        self.d_error[1] = self.error[1] - self.prev_error[1]
        self.d_error[2] = self.error[2] - self.prev_error[2]
        
        self.roll = 1500 + self.error[0] * self.Kp[0] + self.error_sum[0] * self.Ki[0] + self.d_error[0] * self.Kd[0]
        self.pitch = 1500 + self.error[1] * self.Kp[1] + self.error_sum[1] * self.Ki[1] + self.d_error[1] * self.Kd[1]
        self.throttle = 1500 + self.error[2] * self.Kp[2] + self.error_sum[2] * self.Ki[2] + self.d_error[2] * self.Kd[2]
        # print(self.throttle)
        # print(self.Kp, self.Ki, self.Kd)
        # print(self.error[2] * self.Kp[2] + self.error_sum[2] * self.Ki[2] + (self.error[2] - self.prev_error[2]) * self.Kd[2])
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        # self.prev_error[3] = self.error[3]

        
        # This is altitude error which is published on attitude controller
        # self.error[2] = self.set_point[2] - self.altitude
        

        # #Index 0 for latitude and 1 for longitude and 2 for altitude
        # self.error[0] = self.set_point[0] - self.latitude
        # self.error_sum[0]+=self.error[0] * 0.033
        # self.error_difference[0] = (self.error[0] - self.prev_error[0])/0.033
        # output_latitude = self.Kp[0] * self.error[0] + self.Ki[0] * self.error_sum[0] + self.Kd[0] * self.error_difference[0]
        # self.prev_error[0] = self.error[0]

        # # Index 1 for longitude
        # self.error[1] = self.set_point[1] - self.longitude
        # self.error_sum[1]+=self.error[1] * 0.033
        # self.error_difference[1] = (self.error[1] - self.prev_error[1])/0.033
        # output_longitude = self.Kp[1] * self.error[1] + self.Ki[1] * self.error_sum[1] + self.Kd[1] * self.error_difference[1]
        # self.prev_error[1] = self.error[1]
        
        self.edrone_cmd_object.rcRoll = self.roll
        self.edrone_cmd_object.rcPitch = self.pitch
        self.edrone_cmd_object.rcThrottle = self.throttle

        if(self.edrone_cmd_object.rcRoll > 2000):
            self.edrone_cmd_object.rcRoll = 2000
        if(self.edrone_cmd_object.rcRoll < 1000):
            self.edrone_cmd_object.rcRoll = 1000
        if(self.edrone_cmd_object.rcPitch > 2000):
            self.edrone_cmd_object.rcPitch = 2000
        if(self.edrone_cmd_object.rcPitch < 1000):
            self.edrone_cmd_object.rcPitch = 1000
        if(self.edrone_cmd_object.rcThrottle > 2000):
            self.edrone_cmd_object.rcThrottle = 2000
        if(self.edrone_cmd_object.rcThrottle < 1000):
            self.edrone_cmd_object.rcThrottle = 1000
        # Publishing to edrone_cmd topic
        self.drone_cmd_pub.publish(self.edrone_cmd_object)
        self.roll_error_pub.publish(self.error[0])
        self.pitch_error_pub.publish(self.error[1])
        self.throttle_error_pub.publish(self.error[2])
        self.zero_error_pub.publish(0)

        # print(self.lat_to_x(self.drone_position[0]))
        # print(self.long_to_y(self.drone_position[1]))


        if(self.a==False and 0 < self.error[2] < 0.1):
            self.setpoint[0] = 19.0007046575
            self.setpoint[1] = 71.9998955286
            self.a = True
            print("Set Point 1 Changed")

        if(self.a == True and self.b == False and 75 < self.lat_to_x(self.drone_position[0]) < 78.1 and 11 <self.long_to_y(self.drone_position[1]) < 11.1 and self.e == False and self.i == False):
            self.setpoint[2] = 23
            self.e = True
            self.i = True
            print("Set Point 2 Changed")

        if(self.a==True and self.b == False and self.qr_check == True):
            # time.sleep(1.2) 434905 412765 434905
            self.setpoint[2] = 0
            print("Set Point 3 Changed")
            self.b=True
        
        self.function_repeat(True)

        if(self.a == True and self.b == True and self.c == False and self.isTrue == True):
            self.setpoint[2] = 25.0
            print("Set Point 4 Changed")
            self.c = True

        if(self.a == True and self.b == True and self.c == True and self.d == False and self.drone_position[2] >= 25.0 and self.h == False):
            self.Kp[0] = 0.1
            # self.Kp[1] = 0.4
            self.Kd[0] = 200
            # self.Kd[1] = 200
            self.setpoint[0] = self.qr_set_point[0]
            self.setpoint[1] = self.qr_set_point[1]
            self.d = True
            self.h = True
            print("Set Point 5 Changed")

        if(self.a == True and self.b == True and self.c == True and self.d == True and -0.1 < self.lat_to_x(self.drone_position[0]) < 0.1 and -0.1 <self.long_to_y(self.drone_position[1]) < 0.1 and self.d == True and self.f == False):
            self.setpoint[2] = self.qr_set_point[2]
            print("Set Point 6 Changed")
            self.f = True

        if(0.5 < self.range_finder_top[3] < 9 and self.g == False):
            self.setpoint[0] = self.drone_position[0]
            print("Obstcale")
            print(self.setpoint)
            print(self.range_finder_top)
            self.g = True

        if(self.g == True and not (0.3 < self.range_finder_top[3] < 9) and self.k == False):
            self.count+=1
            if(self.count >= 90):
                self.setpoint[0] = self.qr_set_point[0]
                self.setpoint[1] = self.qr_set_point[1]
                print("Set Point Chnaged 7")
                self.Kp[0] = 0.44
                self.Kp[1] = 0.4
                self.Kd[0] = 50.37
                self.Kd[1] = 50.62
                self.k = True
        
        if(self.k == True and self.qr_set_point[2] <= self.drone_position[2] <= 8.5 and self.l == False):
            self.function_repeat(False)
            self.l = True

        if(self.l== True and self.isDrop == True):
            self.setpoint[2] = 10.0





if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(60)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()