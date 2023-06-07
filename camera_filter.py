import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

#constantes
k = 1/240
vel_angular_max = 1.5
minima_diferencia_blancos = 1

def read_image_data(data):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #MASCARAS
    #rojo
    mask = cv2.inRange(frame_HSV, (0,10, 50), (30,255, 200))
    #buen rojo
    mask_red = cv2.inRange(frame_HSV, (160, 131, 89), (189,255, 255))
    #blanco
    mask = cv2.inRange(frame_HSV, (0,0, 220), (190,30, 255))
    
    edges = cv2.Canny(mask, 200, 400)
    #recortar parte de arriba imagen?
    cropped_edges = cv2.bitwise_and(cv_image,cv_image,mask=edges)
    imagen_edges_gris = cv2.cvtColor(cropped_edges, cv2.COLOR_BGR2GRAY)

    frame_RGB = cropped_edges


    #frame_RGB = cv2.bitwise_and(cv_image,cv_image,mask = mask)

    #formas de sacar ruido
    kernel = np.ones((5, 5), np.uint8)
    #frame_RGB = cv2.morphologyEx(frame_RGB, cv2.MORPH_OPEN, kernel)
    frame_RGB = cv2.fastNlMeansDenoisingColored(frame_RGB,None,10,10,7,21)
    

    #primeras 6 columnas
    primer_blanco_izq = -1
    for indice_fila, fila in enumerate(frame_RGB):
        #revisar las 6 columnas adyacentes a la izquierda
        for indice_columna in range(0,6):
            pixel = frame_RGB[-1 - indice_fila][indice_columna]
            #hay color blanco
            if(pixel[0] > 100 and pixel[1] > 100 and pixel[2] > 100):
                primer_blanco_izq = indice_fila
                break
        
        #para salir del otro for
        if primer_blanco_izq > 0:
            break

    #ultimas 6 columnas
    primer_blanco_der = -1
    for indice_fila, fila in enumerate(frame_RGB):
        for indice_columna in range(0,6):
            #hay color blanco
            pixel = frame_RGB[-1 - indice_fila][-1 - indice_columna]
            if(pixel[0] > 100 and pixel[1] > 100 and pixel[2] > 100):
                primer_blanco_der = indice_fila
                break
        
        #para salir del otro for
        if primer_blanco_der > 0:
            break


    

    twist = Twist()
    twist.linear = Vector3(0.1,0,0)

    #para que no afecte el ruido (es una especie de histeresis)
    if abs((primer_blanco_izq - primer_blanco_der)) > minima_diferencia_blancos:
        #si hay mucha diferencia entre donde empieza el color blanco a la izquierda y a la derecha, hacer que robot gire con controlador proporcional el motor
        vel_angular = (primer_blanco_izq - primer_blanco_der) * k
        if vel_angular > vel_angular_max:
            vel_angular = vel_angular_max
            
        if vel_angular < -vel_angular_max:
            vel_angular = -vel_angular_max
            
        twist.angular = Vector3(0, 0, vel_angular)
    
    #delega control
    #if not near_object_flag:
    motor_pub.publish(twist)
    print(twist)

    print(primer_blanco_izq, primer_blanco_der)
    #cv2.imshow("Image window", frame_RGB)
    cv2.imshow("Video", frame_RGB)
    cv2.waitKey(3)

    try:
        #image_pub.publish(CvBridge().cv2_to_imgmsg(frame_RGB, "bgr8"))
        image_pub.publish(CvBridge().cv2_to_imgmsg(cropped_edges, "bgr8"))
    except CvBridgeError as e:
        print(e) 


rospy.init_node('nodo')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
