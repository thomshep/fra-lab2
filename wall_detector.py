import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge, CvBridgeError

import time
import math

esta_girando = False
pixeles_a_evaluar_fila = 8
altura_maxima_pared = 500

minimo_largo_segmento = 100
margen_superior = 360
maximo_tamano_pendiente = 0.1

# def girar():
#     global esta_girando

#     twist = Twist()
#     twist.angular = Vector3(0,0,1)
#     motor_pub.publish(twist)

#     time.sleep(5.5)

#     esta_girando = False

def read_image_data(data):
    global esta_girando, margen_superior, minimo_largo_segmento, maximo_tamano_pendiente
    
    if not esta_girando:

        try:
            cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(frame_HSV, (0,0, 220), (30,190, 255))

        frame_RGB = cv2.bitwise_and(cv_image,cv_image,mask = mask)

        #formas de sacar ruido
        kernel = np.ones((5, 5), np.uint8)
        #frame_RGB = cv2.morphologyEx(frame_RGB, cv2.MORPH_OPEN, kernel)
        #frame_RGB = cv2.fastNlMeansDenoisingColored(frame_RGB,None,10,10,7,21)
        

        #cv2.imshow("Image window", frame_RGB)
        #cv2.waitKey(40)



        # ####### USANDO PUNTOS BLANCOS
        
        # centro_eje_x_imagen = math.floor(frame_RGB.shape[1] / 2)
        


        # for indice_fila, fila in enumerate(frame_RGB):
        #     cantidad_blancos = 0
        #     if indice_fila > altura_maxima_pared:
        #         break
        #     #revisar las 6 columnas adyacentes a la izquierda
        #     for indice_columna in range(centro_eje_x_imagen - math.floor(pixeles_a_evaluar_fila / 2), centro_eje_x_imagen + math.floor(pixeles_a_evaluar_fila / 2)):
        #         pixel = frame_RGB[-1 - indice_fila][indice_columna]
        #         #hay color blanco
        #         if(pixel[0] > 100 and pixel[1] > 100 and pixel[2] > 100):
        #             cantidad_blancos += 1
                    
        #             print(cantidad_blancos)
        #             #para salir del otro for
        #             if cantidad_blancos > pixeles_a_evaluar_fila / 2:
        #                 esta_girando = True
                        
        #                 print(indice_fila)
        #                 print(frame_RGB.shape)
                    

        #                 girar()
        #                 return

        # ####### USANDO SEGMENTOS DE RECTA

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

        def filtrado_segmentos(segmento):
            x1, y1, x2, y2 = segmento[0]
            pendiente = (y2 - y1) / (x2 - x1)

            return abs(pendiente) < maximo_tamano_pendiente and y1 > margen_superior and x2 - x1 > minimo_largo_segmento
        
        #print("len(segmentos)")
        segmentos = list(filter(filtrado_segmentos, segmentos))
        #segmentos = []

        twist = Twist()

        if len(segmentos) > 0: #frenar
            print("frenar")
            print(segmentos)
            wall_pub.publish(True)

        else:
            #Avanzar
            print("avanzar")
            twist.linear = Vector3(0.07,0,0)
            wall_pub.publish(False)

        motor_pub.publish(twist)


        imagen_edges_a_color = cv2.cvtColor(imagen_edges_gris, cv2.COLOR_GRAY2BGR)
        imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos)
        try:
            #image_pub.publish(CvBridge().cv2_to_imgmsg(frame_RGB, "bgr8"))
            image_pub.publish(CvBridge().cv2_to_imgmsg(imagen_edges_a_color, "bgr8"))
        except CvBridgeError as e:
            print(e) 
        #cv2.imshow("Video", imagen_edges_a_color)
       # cv2.waitKey(3)

        

rospy.init_node('detector_pared')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
#motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
motor_pub = rospy.Publisher("dscsd/cmd_vel", Twist, queue_size=20)
wall_pub = rospy.Publisher("controlador_reactivo/hay_pared", Bool, queue_size=10)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data, queue_size=1)
rospy.spin()
