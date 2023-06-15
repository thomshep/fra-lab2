import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math

class Nodo:
    def __init__(self, grafo, posicion_nodo, aristas):
        self.id = len(grafo)
        self.posicion_nodo = posicion_nodo
        self.aristas = aristas
    
    def agregar_arista(self, arista):
        self.aristas.append(arista)


class Arista:
    def __init__(self, nodo_origen, nodo_destino, distancia, orientacion):
        self.nodo_origen = nodo_origen
        self.nodo_destino = nodo_destino
        self.distancia = distancia
        self.orientacion = orientacion


DISTANCIA_MAXIMA_NODOS_CERCA = 0.15


orientaciones = ["N", "E", "S", "O"]

grafo = []
nodo_inicial = Nodo(grafo, (0,0), [])
ultimo_nodo_visitado = nodo_inicial
grafo.append(ultimo_nodo_visitado)

orientacion_actual = "E"

posicion_actual_robot = (0,0)
ultima_posicion_robot = (0,0)


def giro(data):
    global orientacion_actual, orientaciones, grafo, posicion_actual_robot, ultimo_nodo_visitado, ultima_posicion_robot

    copia_posicion_actual_robot = posicion_actual_robot # para que no se vaya modificando mientras procesa todo este codigo

    indice_orientacion_actual = orientaciones.index(orientacion_actual)

    posicion_nodo = (0,0)
    if orientacion_actual == "N" or orientacion_actual == "S":
        posicion_nodo = (ultimo_nodo_visitado.posicion_nodo[0], ultimo_nodo_visitado.posicion_nodo[1] + (copia_posicion_actual_robot[1] - ultima_posicion_robot[1]))
    else:
        posicion_nodo = (ultimo_nodo_visitado.posicion_nodo[0] + (copia_posicion_actual_robot[0] - ultima_posicion_robot[0]), ultimo_nodo_visitado.posicion_nodo[1])
    
    print(posicion_nodo) 
    
    def nodo_cerca_posicion_actual(nodo):
        return abs(nodo.posicion_nodo[0] - posicion_nodo[0]) < DISTANCIA_MAXIMA_NODOS_CERCA and abs(nodo.posicion_nodo[1] - posicion_nodo[1]) < DISTANCIA_MAXIMA_NODOS_CERCA
    nodos_cerca_posicion_actual = list(filter(nodo_cerca_posicion_actual, grafo))
    
    distancia_recorrida = 0
    if orientacion_actual == "N" or orientacion_actual == "S":
        distancia_recorrida = abs(copia_posicion_actual_robot[1] - ultima_posicion_robot[1])
    else:
        distancia_recorrida = abs(copia_posicion_actual_robot[0] - ultima_posicion_robot[0])

    indice_orientacion_inversa = (indice_orientacion_actual + 2) % len(orientaciones)
    orientacion_inversa = orientaciones[indice_orientacion_inversa]

    if len(nodos_cerca_posicion_actual) == 0:
        print("Agrego nodo grafo")
    
        nodo_nuevo = Nodo(grafo, posicion_nodo, [])
        arista_nodo_viejo_nuevo = Arista(ultimo_nodo_visitado, nodo_nuevo, distancia_recorrida, orientacion_actual)

        arista_nodo_nuevo_viejo = Arista(nodo_nuevo, ultimo_nodo_visitado, distancia_recorrida, orientacion_inversa)

        ultimo_nodo_visitado.agregar_arista(arista_nodo_viejo_nuevo)
        nodo_nuevo.agregar_arista(arista_nodo_nuevo_viejo)

        ultimo_nodo_visitado = nodo_nuevo

        grafo.append(nodo_nuevo)
    else:
        nodo_actual = nodos_cerca_posicion_actual[0]
        print("Nodo que reutilizo:")
        print(nodo_actual.posicion_nodo)


        # revisar que no existan ya las aristas que se quieren insertar
        def arista_contiene_nodo_actual(arista):
            return arista.nodo_destino.id == nodo_actual.id

        aristas_con_nodo_actual = list(filter(arista_contiene_nodo_actual, ultimo_nodo_visitado.aristas))

        if len(aristas_con_nodo_actual) == 0:
            arista_nodo_viejo_actual = Arista(ultimo_nodo_visitado, nodo_actual, distancia_recorrida, orientacion_actual)
            arista_nodo_actual_viejo = Arista(nodo_actual, ultimo_nodo_visitado, distancia_recorrida, orientacion_inversa)

            ultimo_nodo_visitado.agregar_arista(arista_nodo_viejo_actual)
            nodo_actual.agregar_arista(arista_nodo_actual_viejo)


        ultimo_nodo_visitado = nodo_actual



    dir_giro = data.data

    indice_nueva_orientacion = indice_orientacion_actual
    if dir_giro == "izq":
        indice_nueva_orientacion = (indice_orientacion_actual - 1) % len(orientaciones)
    if dir_giro == "der":
        indice_nueva_orientacion = (indice_orientacion_actual + 1) % len(orientaciones)
    
    orientacion_actual = orientaciones[indice_nueva_orientacion]
    print("nueva orientacion: " + orientacion_actual)

    ultima_posicion_robot = copia_posicion_actual_robot

    print(len(grafo))
    print()


    



def datos_odometria(data):
    global posicion_actual_robot
    
#    print(data)
    pose = data.pose.pose.position
    posicion_actual_robot = (pose.x, pose.y)



rospy.init_node('navegacion')
rospy.Subscriber("/navegacion/giro", String, giro)
rospy.Subscriber('/odom', Odometry, datos_odometria)

rospy.spin()
