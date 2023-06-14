import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

#constant
MAX_DISTANCE_OBJECT = 0.15
ANGLES_LIDAR_INSPECTED = 10
MAX_DISTANCE_DIFFERENCE = 0.03
#variables
cv_image_cam = None
red_image = None
analizar_imagen = False

#chequea si al menos tiene 3 consecutivos entre 8 y 12 cm
def is_near_object(distances):
    
    for index in range(-ANGLES_LIDAR_INSPECTED + 1, ANGLES_LIDAR_INSPECTED): # -10 grados a 10 grados
        if distances[index-1] > 0 and abs(distances[index-1] - MAX_DISTANCE_OBJECT) <= MAX_DISTANCE_DIFFERENCE \
            and distances[index] >0 and abs(distances[index] - MAX_DISTANCE_OBJECT) <= MAX_DISTANCE_DIFFERENCE \
            and distances[index + 1] > 0 and abs(distances[index+1] - MAX_DISTANCE_OBJECT) <= MAX_DISTANCE_DIFFERENCE:
            return True
    
    return False #cualquier otro caso
                    

def read_sensor(data):
    global analizar_imagen
    if is_near_object(data.ranges):
        #stop
        vel_null = Twist()
        motor_pub.publish(vel_null)
        print(vel_null)

        analizar_imagen = True
    else:
        #devolver control
        twist = Twist()
        twist.linear = Vector3(0.1,0,0)
        motor_pub.publish(twist)

        

def process_object(res):
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
    global cv_image_cam,red_image, analizar_imagen

    if not analizar_imagen:
        return
    

    try:
        cv_image_cam = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    frame_HSV = cv2.cvtColor(cv_image_cam, cv2.COLOR_BGR2HSV)
    #buen rojo
    mask_red = cv2.inRange(frame_HSV, (160, 131, 89), (189,255, 255))
    
    red_image = cv2.bitwise_and(cv_image_cam, cv_image_cam, mask = mask_red)

    #TODO: definir
    mask_yellow = cv2.inRange(frame_HSV, (0, 0, 0), (0,0, 0))
    yellow_image = cv2.bitwise_and(cv_image_cam, cv_image_cam, mask = mask_yellow)

    if process_object(red_image):
        twist = Twist()
        twist.angular = Vector3(0,0, 1)
        motor_pub.publish(twist)

    if process_object(yellow_image):
        twist = Twist()
        twist.angular = Vector3(0,0, -1)
        motor_pub.publish(twist)


rospy.init_node('nodo_red')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
rospy.Subscriber("/scan", LaserScan, read_sensor)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
