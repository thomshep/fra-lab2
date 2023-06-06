import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

there_is_interest_object = False
distance_object = 0.10
range_sensor = 10
near_object_flag = False

#chequea si al menos tiene 3 consecutivos entre 8 y 12 cm
def is_near_object(distances, distance_min):
    first_idx, second_idx, third_idx = -2000, -2000, -2000
    for index in range(-range_sensor, range_sensor + 1): # -10 grados a 10 grados
        if distances[index] != 0 and abs(distances[index] - distance_min) <= 0.3:
            if (first_idx + 1) == index:
                second_idx = index
            elif (second_idx + 1) == index:
                third_idx = index
            else:
                first_idx = index
    
    #chequea si los valores fueron setados            
    if first_idx > -11 and second_idx> -11 and third_idx > -11:
        if second_idx == (first_idx + 1) and third_idx == (second_idx + 1):
            #cumple que son 3 consecutivos
            return True
    
    return False #cualquier otro caso
                    

def read_sensor(data):
    global there_is_interest_object, distance_object
    if there_is_interest_object:
        if is_near_object(data.ranges, distance_object):
            #stop
            near_object_flag = True
            vel_null = Twist(0,0,0)
            motor_pub.publish(vel_null)
            print(vel_null)
        else:
            #devolver control
            near_object_flag = False

def process_red_object(cv_image, frame_hsv, mask_red):
    global there_is_interest_object
    res = cv2.bitwise_and(cv_image, cv_image, mask = mask_red)
    res2 = cv2.cvtColor(frame_hsv, cv2.COLOR_RGBGRAY)
    circles = cv2.HoughCircles(res2,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius = 10,maxRadius = 30)

    ### ALTERNATIVA a HoughCircles ###
    contours, _ = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    THRESHOLD_SIZE = 0.3
    redObjects = []
    for contour in contours:
        area = cv2.contourArea(contour)

        if area > THRESHOLD_SIZE:
            x, y, w, h = cv2.boundingRect(contour)
            red_object = (contour, area, (x + (w / 2), y + (h / 2))) # (contour, area, centro)
            redObjects.append(red_object)

    if redObjects != []:
        ## nos podemos quedar con el objeto rojo mas grande
        pass 

    ### Fin ###
    
    there_is_interest_object = not circles is None
        


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
    mask = cv2.inRange(frame_HSV, (0,0, 220), (30,30, 255))
    
    #detectar objetos rojos
    process_red_object(cv_image,mask_red)
    
    edges = cv2.Canny(mask, 200, 400)
    #recortar parte de arriba imagen?
    cropped_edges = cv2.bitwise_and(cv_image,cv_image,mask=edges)
    imagen_edges_gris = cv2.cvtColor(cropped_edges, cv2.COLOR_BGR2GRAY)

    frame_RGB = cropped_edges

    def detect_line_segments(cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                        np.array([]), minLineLength=8, maxLineGap=4)

        return line_segments
    
    def draw_line_segments(image, line_segments):
        for line in line_segments:
            x1, y1, x2, y2 = line[0]
            image = cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Dibuja una línea verde de grosor 2
        return image

    segmentos = detect_line_segments(imagen_edges_gris)
    imagen_edges_a_color = cv2.cvtColor(imagen_edges_gris, cv2.COLOR_GRAY2BGR)
    imagen_edges_a_color = draw_line_segments(imagen_edges_a_color, segmentos)
    print(imagen_edges_gris)

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


    k = 1/240
    vel_angular_max = 1.5

    twist = Twist()
    twist.linear = Vector3(0.1,0,0)

    #para que no afecte el ruido (es una especie de histeresis)
    if abs((primer_blanco_izq - primer_blanco_der)) > 1:
        #si hay mucha diferencia entre donde empieza el color blanco a la izquierda y a la derecha, hacer que robot gire con controlador proporcional el motor
        vel_angular = (primer_blanco_izq - primer_blanco_der) * k
        if vel_angular > vel_angular_max:
            vel_angular = vel_angular_max
            
        if vel_angular < -vel_angular_max:
            vel_angular = -vel_angular_max
            
        twist.angular = Vector3(0, 0, vel_angular)
    
    #delega control
    if not near_object_flag:
        motor_pub.publish(twist)
        print(twist)

    print(primer_blanco_izq, primer_blanco_der)
    #cv2.imshow("Image window", frame_RGB)
    cv2.imshow("Video", imagen_edges_a_color)
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
rospy.Subscriber("/scan", LaserScan, read_sensor)
rospy.spin()
