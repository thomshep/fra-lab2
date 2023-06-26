import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3
import random
import threading
import time

rospy.init_node('samples')

giro_pub = rospy.Publisher("/navegacion/giro", String, queue_size=20)

def publicar_giro():
    decision = random.randint(0,1)
    if decision == 1:
        print('decide der')
        giro_pub.publish("der")
    else:
        print('decide izq')
        giro_pub.publish("izq")

while True:
    publicar_giro()
    time.sleep(30)
    publicar_giro()
    time.sleep(30)
    publicar_giro()
    time.sleep(6)
    publicar_giro()
    time.sleep(6)

