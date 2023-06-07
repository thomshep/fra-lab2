import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

import time
import math

esta_girando = False
pixeles_a_evaluar_fila = 8
altura_maxima_pared = 500


def girar():
    global esta_girando

    twist = Twist()
    twist.angular = Vector3(0,0,1)
    motor_pub.publish(twist)

    time.sleep(5.5)

    esta_girando = False

def read_image_data(data):
    global esta_girando
    
    if not esta_girando:
        print("entre")
        try:
            cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(frame_HSV, (0,0, 220), (30,30, 255))

        frame_RGB = cv2.bitwise_and(cv_image,cv_image,mask = mask)

        #formas de sacar ruido
        kernel = np.ones((5, 5), np.uint8)
        #frame_RGB = cv2.morphologyEx(frame_RGB, cv2.MORPH_OPEN, kernel)
        frame_RGB = cv2.fastNlMeansDenoisingColored(frame_RGB,None,10,10,7,21)
        
        print("eee")
        #cv2.imshow("Image window", frame_RGB)
        #cv2.waitKey(40)

        print("kkkk")

        centro_eje_x_imagen = math.floor(frame_RGB.shape[1] / 2)
        

        for indice_fila, fila in enumerate(frame_RGB):
            cantidad_blancos = 0
            if indice_fila > altura_maxima_pared:
                break
            #revisar las 6 columnas adyacentes a la izquierda
            for indice_columna in range(centro_eje_x_imagen - math.floor(pixeles_a_evaluar_fila / 2), centro_eje_x_imagen + math.floor(pixeles_a_evaluar_fila / 2)):
                pixel = frame_RGB[-1 - indice_fila][indice_columna]
                #hay color blanco
                if(pixel[0] > 100 and pixel[1] > 100 and pixel[2] > 100):
                    cantidad_blancos += 1
                    
                    print(cantidad_blancos)
                    #para salir del otro for
                    if cantidad_blancos > pixeles_a_evaluar_fila / 2:
                        esta_girando = True
                        
                        print(indice_fila)
                        print(frame_RGB.shape)
                    

                        girar()
                        return
        
        twist = Twist()
        twist.linear = Vector3(0.1,0,0)
        motor_pub.publish(twist)

        

rospy.init_node('nodo')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
