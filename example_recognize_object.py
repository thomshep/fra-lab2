import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

there_is_interest_object = False

def process_red_object(cv_image,frame_hsv,mask_red):
    global there_is_interest_object
    res = cv2.bitwise_and(cv_image,cv_image,mask = mask_red)
    res2 = cv2.cvtColor(frame_hsv,cv2.COLOR_RGBGRAY)
    circles = cv2.HoughCircles(res2,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius = 10,maxRadius = 30)
    
    there_is_interest_object = not circles is None
    
    print(there_is_interest_object)
    
        
def read_image_data(cv_image):
    # try:
    #     cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    # except CvBridgeError as e:
    #     print(e)

    frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #MASCARAS
    #rojo
    mask = cv2.inRange(frame_HSV, (0,10, 50), (30,255, 200))
    #buen rojo
    mask_red = cv2.inRange(frame_HSV, (160, 131, 89), (189,255, 255))
    
    #detectar objetos rojos
    process_red_object(cv_image,frame_HSV,mask_red) 
    
def send_images():
    cap = cv2.VideoCapture(0)
    while(True):
        _,frame = cap.read()
        print("process_image")
        read_image_data(frame)
        
    cv2.destroyAllWindows()

        
        
rospy.init_node('nodo_example')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
#rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
send_images()
rospy.spin()