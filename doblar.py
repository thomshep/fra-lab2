import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

import time
import threading
import random

#constantes
k = 1/240
vel_angular_max = 1.5
minima_diferencia_blancos = 1
margen_superior = 100

esta_girando = False
recien_giro = False

def girar(direccion):
    global esta_girando, recien_giro
    esta_girando = True

    signo = 1

    if direccion == "der":
        signo = -1

    twist = Twist()
    twist.angular = Vector3(0,0, signo * 1)
    motor_pub.publish(twist)

    time.sleep(5.5)
    recien_giro = True

    def habilitar_giro():
        global recien_giro
        recien_giro = False

    t = threading.Timer(5, habilitar_giro)
    t.start()

    esta_girando = False

def read_image_data(data):
    global esta_girando

    if esta_girando or recien_giro:
        return
    
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
    #cv2.imshow("Video", cropped_edges)
    imagen_edges_gris = cv2.cvtColor(cropped_edges, cv2.COLOR_BGR2GRAY)

    frame_RGB = cropped_edges

    def detect_line_segments(cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 30  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                        np.array([]), minLineLength=15, maxLineGap=20)
        if line_segments is None:
            return []

        return line_segments
    
    def region_of_interest(matriz, altura_maxima):
        submatriz = []
        for fila in matriz:
            submatriz.append(fila[len(fila) - altura_maxima:])
        return submatriz

    def draw_line_segments(image, line_segments):
        for line in line_segments:
            x1, y1, x2, y2 = line[0]
            image = cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Dibuja una línea verde de grosor 2
        return image

    segmentos = detect_line_segments(imagen_edges_gris)


    print(imagen_edges_gris.shape)
    def filtrado_segmentos_izquierda(segmento):
        x1, y1, x2, y2 = segmento[0]
        return x1 <= 20 and y1 > margen_superior
    print("len(segmentos)")


    segmentos_izquierda = list(filter(filtrado_segmentos_izquierda, segmentos))
    print(segmentos_izquierda)

    def filtrado_segmentos_derecha(segmento):
        x1, y1, x2, y2 = segmento[0]
        return x2 >= imagen_edges_gris.shape[1] - 20 and y1 > margen_superior
    
    segmentos_derecha = list(filter(filtrado_segmentos_derecha, segmentos))


    

    def evaluar_segmentos_izquierda(segmentos):
        for segmento in segmentos:
            x1, y1, x2, y2 = segmento[0]
            if x2 == x1:
                return
            pendiente = (y2 - y1) / (x2 - x1)
            if abs(pendiente) < 0.1:
                print("girar izq")
                girar("izq")
                return
            else:
                print("No girar izq")

    def evaluar_segmentos_derecha(segmentos):
        for segmento in segmentos:
            x1, y1, x2, y2 = segmento[0]
            if x2 == x1:
                return
            pendiente = (y2 - y1) / (x2 - x1)
            if abs(pendiente) < 0.1:
                print("girar der")
                girar("der")
                return
            else:
                print("No girar der")

    if random.random() > 0.5:
        evaluar_segmentos_izquierda(segmentos_izquierda)
        evaluar_segmentos_derecha(segmentos_derecha)
    else: 
        evaluar_segmentos_derecha(segmentos_derecha)
        evaluar_segmentos_izquierda(segmentos_izquierda)


    twist = Twist()
    twist.linear = Vector3(0.1,0,0)

    print("no girar")


    
    imagen_edges_a_color = cv2.cvtColor(imagen_edges_gris, cv2.COLOR_GRAY2BGR)

    try:
        #image_pub.publish(CvBridge().cv2_to_imgmsg(frame_RGB, "bgr8"))
        image_pub.publish(CvBridge().cv2_to_imgmsg(imagen_edges_a_color, "bgr8"))
    except CvBridgeError as e:
        print(e) 
    return


    # for line in segmentos:
    #         x1, y1, x2, y2 = line[0]
    print(len(segmentos))
    
    

    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos)
    print(imagen_edges_gris)

    #frame_RGB = cv2.bitwise_and(cv_image,cv_image,mask = mask)

    #formas de sacar ruido
    kernel = np.ones((5, 5), np.uint8)
    #frame_RGB = cv2.morphologyEx(frame_RGB, cv2.MORPH_OPEN, kernel)
    frame_RGB = cv2.fastNlMeansDenoisingColored(frame_RGB,None,10,10,7,21)
    
    


    cv2.imshow("Video", imagen_edges_a_color)
    cv2.waitKey(3)

    


rospy.init_node('doblar')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
