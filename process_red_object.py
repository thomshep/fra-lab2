import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

#constant
distance_object = 0.15
range_sensor = 10
#variables
cv_image_cam = None
red_image = None

#chequea si al menos tiene 3 consecutivos entre 8 y 12 cm
def is_near_object(distances, distance_min):
    
    for index in range(-range_sensor + 1, range_sensor): # -10 grados a 10 grados
        if distances[index-1] > 0 and abs(distances[index-1] - distance_min) <= 0.03 \
            and distances[index] >0 and abs(distances[index] - distance_min) <= 0.03 \
            and distances[index + 1]>0 and abs(distances[index+1] - distance_min) <= 0.03:
            return True
    
    return False #cualquier otro caso
                    

def read_sensor(data):
    global distance_object,red_image
    if is_near_object(data.ranges, distance_object) and process_red_object(red_image):
        #stop
        vel_null = Twist(0,0,0)
        motor_pub.publish(vel_null)
        print(vel_null)
    else:
        #devolver control
        twist = Twist()
        twist.linear = Vector3(0.1,0,0)
        motor_pub.publish(twist)

        

def process_red_object(res):
    #res2 = cv2.cvtColor(frame_hsv, cv2.COLOR_RGBGRAY)
    #circles = cv2.HoughCircles(res2,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius = 10,maxRadius = 30)

    ### ALTERNATIVA a HoughCircles ###
    contours, _ = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    THRESHOLD_SIZE = 0.2
    redObjects = []
    for contour in contours:
        area = cv2.contourArea(contour)
        print(area)

        if area > THRESHOLD_SIZE:
            x, y, w, h = cv2.boundingRect(contour)
            red_object = (contour, area, (x + (w / 2), y + (h / 2))) # (contour, area, centro)
            redObjects.append(red_object)
            return True
    
    return False

    if redObjects != []:
        ## nos podemos quedar con el objeto rojo mas grande
        pass 

    #there_is_interest_object = not circles is None
    ### Fin ###
        
        

def read_image_data(data):
    global cv_image_cam,red_image
    try:
        cv_image_cam = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    frame_HSV = cv2.cvtColor(cv_image_cam, cv2.COLOR_BGR2HSV)
    #buen rojo
    mask_red = cv2.inRange(frame_HSV, (160, 131, 89), (189,255, 255))
    
    red_image = cv2.bitwise_and(cv_image_cam, cv_image_cam, mask = mask_red)

rospy.init_node('nodo_red')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
rospy.Subscriber("/scan", LaserScan, read_sensor)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
