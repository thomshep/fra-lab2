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
vel_angular_max = 1
minima_diferencia_blancos = 1

maximo_largo_segmento = 50
minimo_tamano_pendiente = 0.4
margen_superior = 100
k_vel_giro = -1/2
minima_dif_promedios = 0.01

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
    #frame_RGB = cv2.fastNlMeansDenoisingColored(frame_RGB,None,10,10,7,21)
    

    ## USANDO COLOR BLANCO (form vieja)

    # #primeras 6 columnas
    # primer_blanco_izq = -1
    # for indice_fila, fila in enumerate(frame_RGB):
    #     #revisar las 6 columnas adyacentes a la izquierda
    #     for indice_columna in range(0,6):
    #         pixel = frame_RGB[-1 - indice_fila][indice_columna]
    #         #hay color blanco
    #         if(pixel[0] > 100 and pixel[1] > 100 and pixel[2] > 100):
    #             primer_blanco_izq = indice_fila
    #             break
        
    #     #para salir del otro for
    #     if primer_blanco_izq > 0:
    #         break

    # #ultimas 6 columnas
    # primer_blanco_der = -1
    # for indice_fila, fila in enumerate(frame_RGB):
    #     for indice_columna in range(0,6):
    #         #hay color blanco
    #         pixel = frame_RGB[-1 - indice_fila][-1 - indice_columna]
    #         if(pixel[0] > 100 and pixel[1] > 100 and pixel[2] > 100):
    #             primer_blanco_der = indice_fila
    #             break
        
    #     #para salir del otro for
    #     if primer_blanco_der > 0:
    #         break


    

    # twist = Twist()
    # twist.linear = Vector3(0.1,0,0)

    # #para que no afecte el ruido (es una especie de histeresis)
    # if abs((primer_blanco_izq - primer_blanco_der)) > minima_diferencia_blancos:
    #     #si hay mucha diferencia entre donde empieza el color blanco a la izquierda y a la derecha, hacer que robot gire con controlador proporcional el motor
    #     vel_angular = (primer_blanco_izq - primer_blanco_der) * k
    #     if vel_angular > vel_angular_max:
    #         vel_angular = vel_angular_max
            
    #     if vel_angular < -vel_angular_max:
    #         vel_angular = -vel_angular_max
            
    #     twist.angular = Vector3(0, 0, vel_angular)
    
    # #delega control
    # #if not near_object_flag:
    # motor_pub.publish(twist)
    # print(twist)

    # print(primer_blanco_izq, primer_blanco_der)
    # #cv2.imshow("Image window", frame_RGB)



    ##### USANDO SEGMENTOS DE RECTA

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


    def filtrado_segmentos_izquierda(segmento):
        x1, y1, x2, y2 = segmento[0]
        pendiente = (y2 - y1) / (x2 - x1)
        return abs(pendiente) > minimo_tamano_pendiente and pendiente < 0 and x1 <= 20 and y2 - y1 < maximo_largo_segmento and y1 > margen_superior
    
    segmentos_izquierda = list(filter(filtrado_segmentos_izquierda, segmentos))

    def filtrado_segmentos_derecha(segmento):
        x1, y1, x2, y2 = segmento[0]
        pendiente = (y2 - y1) / (x2 - x1)
        return abs(pendiente) > minimo_tamano_pendiente and pendiente > 0 and x2 >= imagen_edges_gris.shape[1] - 20 and y1 > margen_superior
    
    segmentos_derecha = list(filter(filtrado_segmentos_derecha, segmentos))

    promedio_seg_izq = 0
    for segment in segmentos_izquierda:
        x1, y1, x2, y2 = segment[0]
        pendiente = (y2 - y1) / (x2 - x1)
        promedio_seg_izq += pendiente

    if len(segmentos_izquierda) > 0:
        promedio_seg_izq = abs(promedio_seg_izq / len(segmentos_izquierda))

    promedio_seg_der = 0
    for segment in segmentos_derecha:
        x1, y1, x2, y2 = segment[0]
        pendiente = (y2 - y1) / (x2 - x1)
        promedio_seg_der += pendiente

    if len(segmentos_derecha) > 0:
        promedio_seg_der = abs(promedio_seg_der / len(segmentos_derecha))


    twist = Twist()
    twist.linear = Vector3(0.2,0,0)

    velocidad_giro = 0
    if abs(promedio_seg_izq - promedio_seg_der) > minima_dif_promedios:
        velocidad_giro = k_vel_giro * (promedio_seg_izq - promedio_seg_der)
        if velocidad_giro > vel_angular_max:
            velocidad_giro = vel_angular_max
            
        if velocidad_giro < -vel_angular_max:
            velocidad_giro = -vel_angular_max

    if(promedio_seg_izq > promedio_seg_der):
        #print("girar der - izq > der")
        pass
    else:
        pass
        #print("girar izq - der > izq")
    
    twist.angular = Vector3(0,0,velocidad_giro)
    motor_pub.publish(twist)

    print(twist)
    


    imagen_edges_a_color = cv2.cvtColor(imagen_edges_gris, cv2.COLOR_GRAY2BGR)
    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos_izquierda)
    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos_derecha)


    #cv2.imshow("Video", imagen_edges_a_color)
    #cv2.waitKey(3)

    return


    try:
        #image_pub.publish(CvBridge().cv2_to_imgmsg(frame_RGB, "bgr8"))
        image_pub.publish(CvBridge().cv2_to_imgmsg(cropped_edges, "bgr8"))
    except CvBridgeError as e:
        print(e) 


rospy.init_node('seguidor_lineas')
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
#motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
motor_pub = rospy.Publisher("controlador_reactivo/cmd_vel", Twist, queue_size=20)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
