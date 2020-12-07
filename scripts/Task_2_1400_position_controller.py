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
        self.setpoint = [19.0009248718,71.9998318945, 25] # Set points of box
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
        self.qr_set_point = [0,0,0] # Set points recieved from qr code
        self.qr_check = False
        self.isDrop = False
        # self.set_point = [19.0009248718,71.9998955286, 25]
        # self.error_difference = [0,0,0]

        # Flags
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

        # Service 
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
        # Calculate error
        self.error[0] = (self.setpoint[0] - self.drone_position[0])*1000000
        self.error[1] = (self.setpoint[1] - self.drone_position[1])*1000000
        self.error[2] = (self.setpoint[2] - self.drone_position[2])

        # Calculate error sum
        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_sum[2] = self.error_sum[2] + self.error[2]

        # Error difference
        self.d_error[0] = self.error[0] - self.prev_error[0]
        self.d_error[1] = self.error[1] - self.prev_error[1]
        self.d_error[2] = self.error[2] - self.prev_error[2]
        
        # Eqautions
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
        
        self.edrone_cmd_object.rcRoll = self.roll
        self.edrone_cmd_object.rcPitch = self.pitch
        self.edrone_cmd_object.rcThrottle = self.throttle

        # Limiting values
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

        # Publishing
        self.drone_cmd_pub.publish(self.edrone_cmd_object)
        self.roll_error_pub.publish(self.error[0])
        self.pitch_error_pub.publish(self.error[1])
        self.throttle_error_pub.publish(self.error[2])
        self.zero_error_pub.publish(0)

        # print(self.lat_to_x(self.drone_position[0]))
        # print(self.long_to_y(self.drone_position[1]))

        # TO change the setpoint to set points of box
        if(self.a==False and 0 < self.error[2] < 0.1):
            self.setpoint[0] = 19.0007046575
            self.setpoint[1] = 71.9998955286
            self.a = True
            print("Set Point 1 Changed")

        # To bring drone lower to read to qr code
        if(self.a == True and self.b == False and 75 < self.lat_to_x(self.drone_position[0]) < 78.5 and 11 <self.long_to_y(self.drone_position[1]) < 11.5 and self.e == False and self.i == False):
            self.setpoint[2] = 23
            self.e = True
            self.i = True
            print("Set Point 2 Changed")

        # To change setpoint of alitude to pick the box if qr code is scanned
        if(self.a==True and self.b == False and self.qr_check == True):
            self.setpoint[2] = 22.1
            print("Set Point 3 Changed")
            self.b=True
        
        # Request the service to pick the box(If box is pickable or not is checked is function itself).
        self.function_repeat(True)

        # Change altitude set point to 25
        if(self.a == True and self.b == True and self.c == False and self.isTrue == True):
            self.setpoint[2] = 25.0
            print("Set Point 4 Changed")
            self.c = True

        # To assign the coordinates of delivery location scanned from the qr code
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

        # To check whether their is obstacle or not
        if(0.5 < self.range_finder_top[3] < 9 and self.g == False):
            self.setpoint[0] = self.drone_position[0]
            self.setpoint[1] = self.drone_position[1]
            print("Obstcale")
            self.g = True 

        # If there is is a obstacle then the drone stopped by giving deone position as set point and as per BUG2 ALGORITHM to follow straight line from box to location to delivery location and
        #  to switch beteen line follower and wall follower depending on obstcale is present or not.
        # And if obstacle is present along y axis then drone is made to move with the obstacle by reducing the y coordinate that is to move in left direction with building until the ground the clear for flight. 
        if(self.g == True and 0.5 < self.range_finder_top[3] < 9 and self.k == False):
            # Converting cartesian cordinate to longitude and reducing y coordinate by 1 until their is no obstcale in front of the drone.
            self.setpoint[1] = ((self.long_to_y(self.drone_position[1]) - 1) / (-105292.0089353767)) + 72

        # If their are no obstacle after above condition then setpoint are again chnaged to set points of delivery location from qr code.
        if(self.g == True and not (0.3 < self.range_finder_top[3] < 9) and self.k == False):
            self.setpoint[0] = self.qr_set_point[0]
            self.setpoint[1] = self.qr_set_point[1]
            print("Set Point 6 Changed")
            self.Kp[0] = 0.44
            self.Kp[1] = 0.4
            self.Kd[0] = 50.37
            self.Kd[1] = 50.62
            self.k = True

        # To change the altitue set point of drone to delivery location
        if(self.a == True and self.b == True and self.c == True and self.d == True and -0.2 < self.lat_to_x(self.drone_position[0]) < 0.2 and -0.2 <self.long_to_y(self.drone_position[1]) < 0.2 and self.d == True and self.f == False):
            self.setpoint[2] = self.qr_set_point[2]
            print("Set Point 7 Changed")
            self.f = True
        
        # TO deatch the box from the drone
        if(self.k == True and self.qr_set_point[2] <= self.drone_position[2] <= 8.5 and self.l == False):
            self.function_repeat(False)
            self.l = True

        # To fly above the box
        if(self.l== True and self.isDrop == True):
            self.setpoint[2] = 10.0





if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(60)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()