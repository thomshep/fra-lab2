import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3
import random
import threading


#Constantes:
TIEMPO_GIRANDO = 5.5
TIEMPO_EMPEZAR_GIRAR = 4

##ESTADOS:
# AVANZAR
AVANZAR = 0
GIRANDO = 1

# PARED (implica frenar y girar 90)
PARED = 2



estado = AVANZAR

def datos_hay_pared(hay_pared):
    global estado, TIEMPO_GIRANDO

    if estado != AVANZAR:
        return

    if hay_pared == True:
        estado = PARED
        #FRENAR

        print("freno, hay pared")
        twist = Twist()
        motor_pub.publish(twist)

        #se elige direccion de giro aleatoriamente
        signo = 1

        if random.random() > 0.5:
            signo = -1

        twist = Twist()

        twist.angular = Vector3(0,0, signo * 1)
        motor_pub.publish(twist)

        #se espera cierto tiempo mientras gira, durante el cual no se reacciona a los datos sensados
        def termino_girar():
            global estado
            estado = AVANZAR

            print("termino girar despues de pared")
            twist = Twist()
            twist.angular = Vector3(0,0,0)
            
            motor_pub.publish(twist)

        t = threading.Timer(TIEMPO_GIRANDO, termino_girar)
        t.start()
        


def datos_seguidor_lineas(velocidades):
    global estado

    if estado == AVANZAR:
        print("avanzo como me dice seguidor")

        motor_pub.publish(velocidades)


def datos_doblar(doblar):
    global estado
    if estado != AVANZAR:
        return
    
    if doblar == "no":
        return

     
    def girar():
        global TIEMPO_GIRANDO, estado
        estado = GIRANDO

        signo = 1

        if doblar == "der":
            signo = -1

        twist = Twist()
        twist.angular = Vector3(0,0, signo * 1)
        print("giro")

        motor_pub.publish(twist)

        def habilitar_giro():
            global estado
            estado = AVANZAR
            print("termine girar")
            twist = Twist()
            motor_pub.publish(twist)

        t = threading.Timer(TIEMPO_GIRANDO, habilitar_giro)
        t.start()

                    
    t = threading.Timer(TIEMPO_EMPEZAR_GIRAR, girar)
    t.start()


rospy.init_node('controlador_reactivo')


rospy.Subscriber("/controlador_reactivo/doblar", String, datos_doblar)
rospy.Subscriber("/controlador_reactivo/cmd_vel", Twist, datos_seguidor_lineas)
rospy.Subscriber("/controlador_reactivo/hay_pared", Bool, datos_hay_pared)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)


rospy.spin()

