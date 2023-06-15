import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError

#constant
MAX_DISTANCE_OBJECT = 0.15
ANGLES_LIDAR_INSPECTED = 20
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
        print("is near object")
        analizar_imagen = True
    else:
        print("no ve objeto")
        analizar_imagen = False
            

def process_object(res):
    rgb = cv2.cvtColor(res, cv2.COLOR_HSV2RGB)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    THRESHOLD_SIZE = 4000
    for contour in contours:
        area = cv2.contourArea(contour)

        if area > THRESHOLD_SIZE:
            print(area)
            # x, y, w, h = cv2.boundingRect(contour)
            # red_object = (contour, area, (x + (w / 2), y + (h / 2))) # (contour, area, centro)
            # redObjects.append(red_object)
            print("reconocio objeto")

            return True
    
    return False  
        

def read_image_data(data):
    global cv_image_cam,red_image, analizar_imagen

    if not analizar_imagen:
        return
    

    try:
        cv_image_cam = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    frame_HSV = cv2.cvtColor(cv_image_cam, cv2.COLOR_BGR2HSV)
    
    mask_red = cv2.inRange(frame_HSV, (160, 131, 89), (189,255, 255))
    new_red = cv2.inRange(frame_HSV, (136, 72, 94), (186, 255, 255))
    red_image = cv2.bitwise_and(cv_image_cam, cv_image_cam, mask = new_red)

    mask_yellow = cv2.inRange(frame_HSV, (19, 6, 216), (71, 249, 255))
    yellow_image = cv2.bitwise_and(cv_image_cam, cv_image_cam, mask = mask_yellow)

    if process_object(yellow_image): # si ve el minotauro, se mantiene a 10 cms
        twist = Twist()
        twist.angular = Vector3(0,0, -1)
        motor_pub.publish(twist)
        objects_pub.publish("minotauro")
        
    elif process_object(red_image): # si ve rocas, se detiene
        twist = Twist()
        motor_pub.publish(twist)
        objects_pub.publish("roca")

    else:
        objects_pub.publish("no")      



rospy.init_node('procesador_objetos')

objects_pub = rospy.Publisher("/controlador_reactivo/objeto", String, queue_size=10)
image_pub = rospy.Publisher("/mask",Image,queue_size=10)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
rospy.Subscriber("/scan", LaserScan, read_sensor)
rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()
