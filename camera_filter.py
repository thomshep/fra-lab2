import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image


import cv2
from cv_bridge import CvBridge, CvBridgeError


def read_image_data(data):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #MASCARAS
    #rojo
    #mask = cv2.inRange(frame_HSV, (0,10, 50), (30,255, 200))
    #blanco
    mask = cv2.inRange(frame_HSV, (0,0, 220), (30,30, 255))

    frame_RGB = cv2.bitwise_and(cv_image,cv_image,mask = mask)
    

    #formas de sacar ruido
    kernel = np.ones((5, 5), np.uint8)
    #frame_RGB = cv2.morphologyEx(frame_RGB, cv2.MORPH_OPEN, kernel)
    frame_RGB = cv2.fastNlMeansDenoisingColored(frame_RGB,None,10,10,7,21)
    

    #primera columna
    primer_blanco_izq = -1
    for indice, y in enumerate(frame_RGB[0]):
        #hay color blanco
        if(y[0] > 100 and y[1] > 100 and y[2] > 100):
            primer_blanco_izq = indice
            print(primer_blanco_izq)
            break

    #ultima columna
    primer_blanco_der = -1
    for indice, y in enumerate(frame_RGB[-4]):
        print(y)
        #hay color blanco
        if(y[0] > 100 and y[1] > 100 and y[2] > 100):
            primer_blanco_der = indice
            print(primer_blanco_der)
            break
    
    print(primer_blanco_izq, primer_blanco_der)
    #cv2.imshow("Image window", frame_RGB)
    #cv2.waitKey(3)

    try:
        image_pub.publish(CvBridge().cv2_to_imgmsg(frame_RGB, "bgr8"))
    except CvBridgeError as e:
        print(e) 


rospy.init_node('nodo')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()