import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import networkx as nx
import matplotlib.pyplot as plt
import time

import signal

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


def camino_mas_corto(G, nodo_inicial, nodo_final):
    shortest_path = nx.dijkstra_path(G, nodo_inicial.id, nodo_final.id, weight='weight')

    # Obtener la distancia del camino más corto
    shortest_distance = nx.dijkstra_path_length(G, nodo_inicial.id, nodo_final.id, weight='weight')

    print(f"Camino: {shortest_path} => Distancia: {shortest_distance}")


DISTANCIA_MAXIMA_NODOS_CERCA = 0.40

#define orientaciones de signo segun
#E,O,N,S
signal_orientation = {
    "N":1,
    "S":-1,
    "E":1,
    "O":-1
}
orientaciones = ["N", "E", "S", "O"]

grafo = []
nodo_inicial = Nodo(grafo, (0,0), [])
ultimo_nodo_visitado = nodo_inicial
grafo.append(ultimo_nodo_visitado)

orientacion_actual = "E"

posicion_actual_robot = (0,0)
ultima_posicion_robot = (0,0)

tiempo_posicion_actual = time.time()
tiempo_ultima_posicion = time.time()

G = nx.Graph()
G.add_node(nodo_inicial.id, pos=(0, 0))

acaba_de_girar = False


def giro(data):
    global orientacion_actual, orientaciones, grafo, posicion_actual_robot, ultimo_nodo_visitado, ultima_posicion_robot, G, acaba_de_girar,tiempo_posicion_actual,tiempo_ultima_posicion

    if acaba_de_girar:
        return

    tiempo_posicion_actual = time.time()
    copia_tiempo_posicion_actual = tiempo_posicion_actual
    #copia_posicion_actual_robot =  posicion_actual_robot # para que no se vaya modificando mientras procesa todo este codigo

    indice_orientacion_actual = orientaciones.index(orientacion_actual)

    posicion_nodo = (0,0)
    if orientacion_actual == "N" or orientacion_actual == "S":
        posicion_nodo = (ultimo_nodo_visitado.posicion_nodo[0], ultimo_nodo_visitado.posicion_nodo[1] + signal_orientation[orientacion_actual]*(copia_tiempo_posicion_actual - tiempo_ultima_posicion))
    else:
        posicion_nodo = (ultimo_nodo_visitado.posicion_nodo[0] + signal_orientation[orientacion_actual]*(copia_tiempo_posicion_actual - tiempo_ultima_posicion), ultimo_nodo_visitado.posicion_nodo[1])
    
    print(posicion_nodo) 
    print("posicion actual robot:" )
    print( copia_tiempo_posicion_actual)
    print("ultima posicion robot:" )
    print( tiempo_ultima_posicion)

    
    def nodo_cerca_posicion_actual(nodo):
        return abs(nodo.posicion_nodo[0] - posicion_nodo[0]) < DISTANCIA_MAXIMA_NODOS_CERCA and abs(nodo.posicion_nodo[1] - posicion_nodo[1]) < DISTANCIA_MAXIMA_NODOS_CERCA
    nodos_cerca_posicion_actual = list(filter(nodo_cerca_posicion_actual, grafo))
    
    distancia_recorrida = 0
    if orientacion_actual == "N" or orientacion_actual == "S":
        distancia_recorrida = abs(copia_tiempo_posicion_actual - tiempo_ultima_posicion)
    else:
        distancia_recorrida = abs(copia_tiempo_posicion_actual - tiempo_ultima_posicion)
    
    print("distancia:")
    print(distancia_recorrida)
    indice_orientacion_inversa = (indice_orientacion_actual + 2) % len(orientaciones)
    orientacion_inversa = orientaciones[indice_orientacion_inversa]

   

    if len(nodos_cerca_posicion_actual) == 0:
        print("Agrego nodo grafo")
    
        nodo_nuevo = Nodo(grafo, posicion_nodo, [])
        G.add_node(nodo_nuevo.id, pos=nodo_nuevo.posicion_nodo)

        arista_nodo_viejo_nuevo = Arista(ultimo_nodo_visitado, nodo_nuevo, distancia_recorrida, orientacion_actual)

        arista_nodo_nuevo_viejo = Arista(nodo_nuevo, ultimo_nodo_visitado, distancia_recorrida, orientacion_inversa)

        G.add_edge(nodo_nuevo.id, ultimo_nodo_visitado.id, label=orientacion_actual, weight=distancia_recorrida)

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

            G.add_edge(nodo_actual.id, ultimo_nodo_visitado.id, label=orientacion_actual, weight=distancia_recorrida)

            ultimo_nodo_visitado.agregar_arista(arista_nodo_viejo_actual)
            nodo_actual.agregar_arista(arista_nodo_actual_viejo)


        ultimo_nodo_visitado = nodo_actual


    tiempo_ultima_posicion = copia_tiempo_posicion_actual

    print("orientacion actual: " + orientacion_actual)

    # Calculo nueva orientacion
    indice_nueva_orientacion = indice_orientacion_actual
    print("indice orient actual: " + str(indice_orientacion_actual))
    dir_giro = data.data
    print(dir_giro)
    if dir_giro == "izq":
        
        indice_nueva_orientacion = (indice_orientacion_actual - 1) % len(orientaciones)
    if dir_giro == "der":
        indice_nueva_orientacion = (indice_orientacion_actual + 1) % len(orientaciones)
    print(indice_nueva_orientacion)
    orientacion_actual = orientaciones[indice_nueva_orientacion]

    
    print("nueva orientacion: " + orientacion_actual)


    print(len(grafo))
    print()


    acaba_de_girar = True

    import threading
    def cambiar_acaba_de_girar():
        global acaba_de_girar
        acaba_de_girar = False

    hilo = threading.Timer(2, cambiar_acaba_de_girar)
    hilo.start()


    



def datos_odometria(data):
    global posicion_actual_robot
    
#    print(data)
    pose = data.pose.pose.position
    posicion_actual_robot = (pose.x, pose.y)



rospy.init_node('navegacion')
rospy.Subscriber("/navegacion/giro", String, giro)
rospy.Subscriber('/odom', Odometry, datos_odometria)

def signal_handler(signal, frame):
    global G
    print("se esta cortando programa")
    # Obtener posiciones de los nodos
    pos = nx.get_node_attributes(G, 'pos')

    # Dibujar el grafo
    plt.figure(figsize=(8, 8))
    nx.draw_networkx_nodes(G, pos=pos, node_color='r', node_size=200)
    nx.draw_networkx_edges(G, pos=pos, edge_color='b')
    nx.draw_networkx_labels(G, pos=pos)
    plt.axis('off')

    # Guardar la imagen en formato PNG
    plt.savefig('grafo.png', format='png')

    #camino_mas_corto()

#esto se va a ejecutar cuando se corte el programa: para en ese momento almacenar el grafo generado
signal.signal(signal.SIGINT, signal_handler)

rospy.spin()
