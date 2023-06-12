import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError

import time
import threading
import random

#constantes
k = 1/240
vel_angular_max = 1
minima_diferencia_blancos = 1
margen_superior = 70

esta_girando = False
recien_giro = False

maximo_tamano_pendiente = 0.1
maximo_largo_segmento = 50
minimo_tamano_pendiente_vertical = 0.4
tiempo_empezar_girar = 2.5 #5
tiempo_girando = 5.5 #6
diferencia_maxima_segmentos = 10

contador = 0

# 0 es evalua a donde girar, 1 es ya decidio a donde girar y esta esperando a llegar, 2 es esta girando
estado = 0

def girar(direccion):
    global tiempo_girando


    signo = 1

    if direccion == "der":
        signo = -1

    twist = Twist()
    twist.angular = Vector3(0,0, signo * 1)
    motor_pub.publish(twist)



    def habilitar_giro():
       global estado
       print("deje de girar")
       estado = 0

    t = threading.Timer(tiempo_girando, habilitar_giro)
    t.start()



def read_image_data(data):
    global esta_girando, minimo_tamano_pendiente_vertical

    if estado != 0:
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

    # kernel = np.ones((5, 5), np.uint8)
    # frame_RGB = cv2.morphologyEx(frame_RGB, cv2.MORPH_OPEN, kernel)

    

    def detect_line_segments(cropped_edges):
        # tunevaluar_segmentosing min_threshold, minLineLength, maxLineGap is a trial and error process by hand
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


    #print(imagen_edges_gris.shape)
    def filtrado_segmentos_izquierda_horizontales(segmento):
        x1, y1, x2, y2 = segmento[0]
        pendiente = (y2 - y1) / (x2 - x1)
        return abs(pendiente) < maximo_tamano_pendiente and x1 <= 20 and x2 - x1 < maximo_largo_segmento and y1 > margen_superior
    #print("len(segmentos)")


    segmentos_izquierda = list(filter(filtrado_segmentos_izquierda_horizontales, segmentos))
    #print(segmentos_izquierda)

    def filtrado_segmentos_derecha_horizontales(segmento):
        x1, y1, x2, y2 = segmento[0]
        pendiente = (y2 - y1) / (x2 - x1)
        return abs(pendiente) < maximo_tamano_pendiente and x2 >= imagen_edges_gris.shape[1] - 20 and x2 - x1 < maximo_largo_segmento and y1 > margen_superior

    
    segmentos_derecha = list(filter(filtrado_segmentos_derecha_horizontales, segmentos))


    def filtrado_segmentos_izquierda_verticales(segmento):
        x1, y1, x2, y2 = segmento[0]
        pendiente = (y2 - y1) / (x2 - x1)
        return abs(pendiente) > minimo_tamano_pendiente_vertical and x1 >= 20 and x1 <= 60 and y1 > margen_superior

    segmentos_izquierda_verticales = list(filter(filtrado_segmentos_izquierda_verticales, segmentos))
    if len(segmentos_izquierda_verticales) > 0:
        segmentos_izquierda_verticales = [max(segmentos_izquierda_verticales, key=lambda x: x[0][1])]
    
    def filtrado_segmentos_derecha_verticales(segmento):
        x1, y1, x2, y2 = segmento[0]
        pendiente = (y2 - y1) / (x2 - x1)
        return abs(pendiente) > minimo_tamano_pendiente_vertical and x2 <= imagen_edges_gris.shape[1] - 20 and x2 >= imagen_edges_gris.shape[1] - 60 and y1 > margen_superior

    segmentos_derecha_verticales = list(filter(filtrado_segmentos_derecha_verticales, segmentos))
    if len(segmentos_derecha_verticales) > 0:
        segmentos_derecha_verticales = [max(segmentos_derecha_verticales, key=lambda x: x[0][1])]

    def evaluar_segmentos(segmentos, dir_giro, segmentos_verticales):
        global estado, tiempo_empezar_girar, contador
        if len(segmentos_verticales) > 0 and len(segmentos) > 0:
            segmento_vertical = segmentos_verticales[0]

            ##TODO: hacer de 0 a 60 el filtrado de verticales y mirar que coincidan en los x tambien
            def coincide_extremo_segmento_vertical_con_horizontal(segmento):
                global diferencia_maxima_segmentos
                x1_h, y1_h, x2_h, y2_h = segmento[0]
                x1_v, y1_v, x2_v, y2_v = segmento_vertical[0]
                #print("horiz primero")
                #print(segmento[0])
                #print(segmento_vertical[0])
                if dir_giro == "der":
                    return abs(y1_h - y2_v) <= diferencia_maxima_segmentos
                else:
                    return abs(y1_v - y2_h) <= diferencia_maxima_segmentos 



            coincidencias_segmentos = list(filter(coincide_extremo_segmento_vertical_con_horizontal, segmentos))

            if len(coincidencias_segmentos) > 0:
                doblar_pub.publish(dir_giro)
                
                contador += 1
                print(contador)
                print("giro " + dir_giro)
                estado = 1
                print("espero para girar")

                def empezar_girar():
                    global estado
                    estado = 2
                    print("empece a girar")
                    girar(dir_giro)
                    
                    
                t = threading.Timer(tiempo_empezar_girar, empezar_girar)
                t.start()

                return
        
        doblar_pub.publish("no")


    if random.random() > 0.5:
        evaluar_segmentos(segmentos_izquierda, "izq", segmentos_izquierda_verticales)
        evaluar_segmentos(segmentos_derecha, "der", segmentos_derecha_verticales)
    else: 
        evaluar_segmentos(segmentos_derecha, "der", segmentos_derecha_verticales)
        evaluar_segmentos(segmentos_izquierda, "izq", segmentos_izquierda_verticales)


    twist = Twist()
    twist.linear = Vector3(0.1,0,0)

    if estado != 2:
      #print("avanzar")
      motor_pub.publish(twist)


    
    imagen_edges_a_color = cv2.cvtColor(imagen_edges_gris, cv2.COLOR_GRAY2BGR)
    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos_izquierda)
    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos_derecha)

    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos_derecha_verticales)
    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos_izquierda_verticales)

    cv2.imshow("Video", imagen_edges_a_color)
    cv2.waitKey(3)

    



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
    
    
    try:
        #image_pub.publish(CvBridge().cv2_to_imgmsg(frame_RGB, "bgr8"))
        image_pub.publish(CvBridge().cv2_to_imgmsg(cropped_edges, "bgr8"))
    except CvBridgeError as e:
        print(e) 

    cv2.imshow("Video", imagen_edges_a_color)
    cv2.waitKey(3)

    


rospy.init_node('doblar')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
#motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
motor_pub = rospy.Publisher("sss/cmd_vel", Twist, queue_size=20)
doblar_pub = rospy.Publisher("controlador_reactivo/doblar", String, queue_size=10)

rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
