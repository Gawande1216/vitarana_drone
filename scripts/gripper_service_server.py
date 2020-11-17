#!/usr/bin/env python

import rospy
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String

class edrone_gripper():

    def __init__(self):
        rospy.init_node('gripper_service_server')
        self.service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        self.service.wait_for_service()

        self.gripper_check = "False"
        # gripper_service = rospy.Service('/edrone/activate_gripper', Gripper, self.gripper_service_server_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        # print("c")

    
    def function_repeat(self):
        if(self.gripper_check == "True"):
            service_response = self.service(True)
            print(service_response.result + " Parcel picked up")
        else:
            service_response = self.service(False)
            print(service_response.result +  " Parcel not picked up")

    def gripper_check_callback(self, msg):
        self.gripper_check = msg.data
        # print(msg.data)

if __name__ == "__main__":
    gripper = edrone_gripper()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        gripper.function_repeat()
        r.sleep()

