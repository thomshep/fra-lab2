import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3
import random
import threading


#Constantes:
TIEMPO_GIRANDO = 5.5
TIEMPO_EMPEZAR_GIRAR = 1.5
TIEMPO_ESPERA_DETECCIONES_GIRAR = 1
DETECCIONES_NECESARIAS_GIRAR = 3

##ESTADOS:
# AVANZAR
AVANZAR = 0
GIRANDO = 1

# PARED (implica frenar y girar 90)
PARED = 2
DETENIDO = 3



estado = AVANZAR
cantidad_girar_izq_detectados = 0
cantidad_girar_der_detectados = 0


def datos_hay_pared(hay_pared):
    global estado, TIEMPO_GIRANDO

    if estado != AVANZAR:
        return

    hay_pared = hay_pared.data

    if hay_pared == True:
        estado = PARED
        #FRENAR

        print("freno, hay pared")
        twist = Twist()
        motor_pub.publish(twist)

        #se elige direccion de giro aleatoriamente
        signo = 1
        doblar = "izq"

        if random.random() > 0.5:
            signo = -1
            doblar = "der"

        twist = Twist()

        twist.angular = Vector3(0,0, signo * 1)
        motor_pub.publish(twist)

        giro_pub.publish(doblar)

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
        
def datos_process_objects(objeto):
    global estado
    if estado != AVANZAR:
        return

    if objeto == "no":
        estado = AVANZAR
        return

    if objeto == "roca": # freno
        estado = DETENIDO

        twist = Twist()
        motor_pub.publish(twist)
        import time
        time.sleep(2)
        print("veo roca -> freno")

    elif objeto == "minotauro":
        estado = DETENIDO

        twist = Twist()
        motor_pub.publish(twist)
        print("veo minotauro -> freno")


def datos_seguidor_lineas(velocidades):
    global estado

    if estado == DETENIDO:
        return
    
    if estado == AVANZAR:
        print("avanzo como me dice seguidor")

        motor_pub.publish(velocidades)


def datos_doblar(doblar):
    global estado, cantidad_girar_der_detectados, cantidad_girar_izq_detectados

    if estado != AVANZAR:
        return

    doblar = doblar.data
    
    if doblar == "no":
        return
    
    giro_pub.publish(doblar)


    def girar():
        global TIEMPO_GIRANDO, estado
        estado = GIRANDO

        signo = 1

        if doblar == "der":
            signo = -1

        twist = Twist()
        twist.angular = Vector3(0,0, signo * 1)


        # def segundo_giro():
        #     twist = Twist()
        #     twist.angular = Vector3(0,0, signo * 1)
        #     motor_pub.publish(twist)

        # hilo1 = threading.Timer(1, segundo_giro)
        # hilo1.start()

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

    
    if doblar == "der":
        print("acumulo der")
        cantidad_girar_der_detectados += 1
        if cantidad_girar_der_detectados == 1: #es el primer detectado, doy tiempo maximo para que lleguen otras detecciones
            def reiniciar_contador_der():
                global cantidad_girar_der_detectados
                cantidad_girar_der_detectados = 0
                print("reinicio der")
            timer = threading.Timer(TIEMPO_ESPERA_DETECCIONES_GIRAR, reiniciar_contador_der)
            timer.start()

    else:
        cantidad_girar_izq_detectados += 1
        print("acumulo izq")
        if cantidad_girar_izq_detectados == 1: #es el primer detectado, doy tiempo maximo para que lleguen otras detecciones
            def reiniciar_contador_izq():
                global cantidad_girar_izq_detectados
                cantidad_girar_izq_detectados = 0
                print("reinicio izq")
            timer = threading.Timer(TIEMPO_ESPERA_DETECCIONES_GIRAR, reiniciar_contador_izq)
            timer.start()

    if cantidad_girar_der_detectados == DETECCIONES_NECESARIAS_GIRAR or cantidad_girar_izq_detectados == DETECCIONES_NECESARIAS_GIRAR:             
        t = threading.Timer(TIEMPO_EMPEZAR_GIRAR, girar)
        t.start()


rospy.init_node('controlador_reactivo')


rospy.Subscriber("/controlador_reactivo/doblar", String, datos_doblar)
rospy.Subscriber("/controlador_reactivo/cmd_vel", Twist, datos_seguidor_lineas)
rospy.Subscriber("/controlador_reactivo/objeto", String, datos_process_objects)
rospy.Subscriber("/controlador_reactivo/hay_pared", Bool, datos_hay_pared)
motor_pub = rospy.Publisher("dynamixel_workbench/cmd_vel", Twist, queue_size=20)
giro_pub = rospy.Publisher("/navegacion/giro", String, queue_size=20)


rospy.spin()

