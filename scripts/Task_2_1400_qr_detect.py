#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
from std_msgs.msg import Float64
from std_msgs.msg import String
# from PIL import Image

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		# Publish to postion_controller
		self.qr_latitude_pub = rospy.Publisher('/qr_latitude', String, queue_size=1)
		self.qr_longitude_pub = rospy.Publisher('/qr_longitude', String, queue_size=1)
		self.qr_altitude_pub = rospy.Publisher('/qr_altitude', String, queue_size=1)
		self.qr_check_pub = rospy.Publisher('/qr_check', String, queue_size=1)
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()

	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			d = decode(self.img)
			if(d):
				output = d[0].data.decode()
				# Publish to postion_controller
				self.qr_latitude_pub.publish(output[0:7])
				self.qr_longitude_pub.publish(output[8:15])
				self.qr_altitude_pub.publish(output[16:20])
				# Publish to position_controller to confirm that qr code is scanned
				self.qr_check_pub.publish("True")
				print(output)
		except CvBridgeError as e:
			print(e)
			return

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()