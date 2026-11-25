#!/usr/bin/env python3
"""
mision_autonoma.py  -  Nexo (MentorPi A1, Ackermann)

Ciclo 100 % autonomo:
  AGV Pool -> Inicio -> espera pedido (MQTT, max. 30 s; si no llega, objeto aleatorio)
  -> Almacen del objeto -> Inicio -> espera pedido -> ...

  Objeto 1 -> Alm 1 | Objeto 2 -> Alm 2 | Objeto 3 -> Alm 3

Antes de cada tramo calcula la ruta con Nav2 y revisa la bateria:
  energia = K_PCT_PER_M * distancia + RESERVA_PCT
  (hacia un almacen suma tambien el tramo almacen -> cargador mas cercano,
   para no quedarse varado despues de entregar).
Si no alcanza, va al cargador mas cercano al que si llega, espera la carga
y vuelve a evaluar el tramo pendiente.

Entradas (FT Optix -> MQTT -> mqtt_to_ros):
  /task      std_msgs/Int32   objeto 1, 2 o 3 (pedido)
  /instruct  std_msgs/String  'pausa' | 'reanudar' | 'cargado'
  /estop     std_msgs/Bool    true = paro de emergencia, false = continuar

Salidas (ros_to_mqtt -> MQTT -> FT Optix):
  /nexo/mission/state        std_msgs/String  estado actual
  /nexo/mission/object       std_msgs/Int32   objeto en curso (0 = ninguno)
  /nexo/mission/destination  std_msgs/String  destino actual
  /nexo/mission/order_source std_msgs/String  'MQTT' | 'ALEATORIO'
  /nexo/mission/deliveries   std_msgs/Int32   entregas completadas
  /nexo/mission/event        std_msgs/String  eventos en texto

Uso:  ros2 run nexo_mentorpi mision_autonoma
No correr al mismo tiempo que smac_navigator.
"""
import math
import random
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float32, Int32, String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ---------------------------------------------------------------- configuracion
# (nombre, x, y, yaw)  yaw=None -> orientarse en la direccion de llegada
ESTACIONES = {
    '1': ('Est C1',   1.65,  4.10,  None),
    '2': ('Est C2',   0.49,  0.003, None),
    '3': ('Est C3',   7.91, -0.004, None),
    '4': ('Alm 1',    3.83,  4.32,  None),
    '5': ('Alm 2',    3.82,  0.75,  None),
    '6': ('Alm 3',    8.98,  1.12,  None),
    '7': ('AGV Pool', 0.00,  0.00,  0.0),
    '8': ('Inicio',   0.79,  4.14,  None),
}
ESTACIONES_CARGA = ('1', '2', '3')
INICIO = '8'
AGV_POOL = '7'
OBJETO_A_ALMACEN = {1: '4', 2: '5', 3: '6'}

T_ESPERA_PEDIDO = 30.0        # s esperando pedido MQTT en Inicio
T_DESCARGA = 5.0              # s detenido en el almacen "descargando"
K_PCT_PER_M = 4.0             # % de bateria por metro recorrido
RESERVA_PCT = 20.0            # % minimo que debe sobrar al llegar
PCT_CARGADO = 80.0            # % con el que sale del cargador
TOLERANCIA_LLEGADA_M = 0.15   # margen para dar por llegada la meta
REINTENTOS_NAV = 2            # reintentos si Nav2 falla un tramo
REINTENTOS_RUTA = 3           # reintentos si el planner no encuentra ruta


# ---------------------------------------------------------------- utilidades
def crear_pose(x, y, yaw=None, desde=None):
    """Meta en 'map', sin estampa de tiempo (TF usa la transformacion mas reciente)."""
    if yaw is None:
        if desde is None or desde[0] is None:
            yaw = 0.0
        else:
            yaw = math.atan2(y - desde[1], x - desde[0])
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def longitud_ruta(path):
    if path is None or len(path.poses) < 2:
        return None
    total = 0.0
    for a, b in zip(path.poses[:-1], path.poses[1:]):
        total += math.hypot(b.pose.position.x - a.pose.position.x,
                            b.pose.position.y - a.pose.position.y)
    return total


def nombre(key):
    return ESTACIONES[key][0]


# ---------------------------------------------------------------- mision
class Mision:

    def __init__(self):
        self.nav = BasicNavigator(node_name='mision_autonoma')
        n = self.nav

        self.pct = None
        self.status = 'DESCONOCIDO'
        self.robot = {'x': None, 'y': None}
        self.pedido = None
        self.estop = False
        self.pausa = False
        self.cargado = False
        self.entregas = 0
        self.ubicacion = AGV_POOL

        self.s_estado = 'INICIANDO'
        self.s_objeto = 0
        self.s_destino = ''
        self.s_origen = ''

        n.create_subscription(Float32, '/nexo/battery/percentage', self.cb_pct, 10)
        n.create_subscription(String, '/nexo/battery/status', self.cb_status, 10)
        qos_amcl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, qos_amcl)
        n.create_subscription(Int32, '/task', self.cb_task, 10)
        n.create_subscription(String, '/instruct', self.cb_instruct, 10)
        n.create_subscription(Bool, '/estop', self.cb_estop, 10)

        self.pub_estado = n.create_publisher(String, '/nexo/mission/state', 10)
        self.pub_objeto = n.create_publisher(Int32, '/nexo/mission/object', 10)
        self.pub_destino = n.create_publisher(String, '/nexo/mission/destination', 10)
        self.pub_origen = n.create_publisher(String, '/nexo/mission/order_source', 10)
        self.pub_entregas = n.create_publisher(Int32, '/nexo/mission/deliveries', 10)
        self.pub_evento = n.create_publisher(String, '/nexo/mission/event', 10)

        # Re-publica el estado completo cada 2 s para el tablero de FT Optix
        n.create_timer(2.0, self.publicar_todo)

    # ---- callbacks
    def cb_pct(self, msg):
        self.pct = msg.data

    def cb_status(self, msg):
        self.status = msg.data

    def cb_amcl(self, msg):
        self.robot['x'] = msg.pose.pose.position.x
        self.robot['y'] = msg.pose.pose.position.y

    def cb_task(self, msg):
        if msg.data in OBJETO_A_ALMACEN:
            self.pedido = msg.data
            self.evento(f'Pedido recibido por MQTT: objeto {msg.data}')
        else:
            self.evento(f'Pedido ignorado: objeto {msg.data} no existe (usa 1, 2 o 3)')

    def cb_instruct(self, msg):
        orden = msg.data.strip().lower()
        if orden == 'pausa':
            self.pausa = True
            self.evento('Instruccion: pausa')
        elif orden == 'reanudar':
            self.pausa = False
            self.evento('Instruccion: reanudar')
        elif orden == 'cargado':
            self.cargado = True
            self.evento('Instruccion: cargado')
        else:
            self.evento(f'Instruccion desconocida: {msg.data}')

    def cb_estop(self, msg):
        nuevo = bool(msg.data)
        if nuevo != self.estop:
            self.estop = nuevo
            self.evento('PARO DE EMERGENCIA' if nuevo else 'Paro liberado')

    # ---- publicacion
    def publicar_todo(self):
        self.pub_estado.publish(String(data=self.s_estado))
        self.pub_objeto.publish(Int32(data=self.s_objeto))
        self.pub_destino.publish(String(data=self.s_destino))
        self.pub_origen.publish(String(data=self.s_origen))
        self.pub_entregas.publish(Int32(data=self.entregas))

    def estado(self, s):
        if s != self.s_estado:
            self.s_estado = s
            self.pub_estado.publish(String(data=s))
            print(f'[ESTADO] {s}', flush=True)

    def destino(self, s):
        self.s_destino = s
        self.pub_destino.publish(String(data=s))

    def objeto(self, obj, origen=''):
        self.s_objeto = obj
        self.s_origen = origen
        self.pub_objeto.publish(Int32(data=obj))
        self.pub_origen.publish(String(data=origen))

    def evento(self, s):
        self.pub_evento.publish(String(data=s))
        print(f'[EVENTO] {s}', flush=True)

    def texto_bateria(self):
        return 'sin lectura' if self.pct is None else f'{self.pct:.1f}%'

    # ---- esperas
    def spin(self, seg):
        fin = time.time() + seg
        while rclpy.ok() and time.time() < fin:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def esperar_si_detenido(self):
        """Bloquea mientras haya paro de emergencia o pausa."""
        previo = self.s_estado
        detenido = False
        while rclpy.ok() and (self.estop or self.pausa):
            detenido = True
            self.estado('PARO_EMERGENCIA' if self.estop else 'PAUSA')
            self.spin(0.2)
        if detenido:
            self.estado(previo)

    def esperar_nav2(self):
        self.estado('ESPERANDO_NAV2')
        while not self.nav.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            print('Esperando servidor /navigate_to_pose...', flush=True)
        self.evento('Nav2 listo. Esperando localizacion de AMCL (/amcl_pose)...')
        while rclpy.ok() and self.robot['x'] is None:
            self.spin(0.5)
        self.evento(f"Robot localizado en ({self.robot['x']:.2f}, {self.robot['y']:.2f})")
        if self.pct is None:
            self.evento('AVISO: sin lectura de bateria (corre battery_manager)')

    # ---- rutas
    def distancia(self, desde_key, hasta_key):
        """Largo de la ruta de Nav2. desde_key=None -> desde la pose actual del robot."""
        _, x, y, yaw = ESTACIONES[hasta_key]
        if desde_key is None:
            goal = crear_pose(x, y, yaw, desde=(self.robot['x'], self.robot['y']))
            path = self.nav.getPath(PoseStamped(), goal, use_start=False)
        else:
            _, sx, sy, _ = ESTACIONES[desde_key]
            rumbo = math.atan2(y - sy, x - sx)
            start = crear_pose(sx, sy, rumbo)
            goal = crear_pose(x, y, yaw, desde=(sx, sy))
            path = self.nav.getPath(start, goal, use_start=True)
        return longitud_ruta(path)

    def cargador_desde(self, key):
        """Cargador mas cercano (en linea recta) a una estacion y largo de su ruta."""
        _, x, y, _ = ESTACIONES[key]
        c = min(ESTACIONES_CARGA,
                key=lambda k: math.hypot(ESTACIONES[k][1] - x, ESTACIONES[k][2] - y))
        if c == key:
            return c, 0.0
        d = self.distancia(key, c)
        if d is None:
            d = 1.3 * math.hypot(ESTACIONES[c][1] - x, ESTACIONES[c][2] - y)
        return c, d

    def elegir_cargador(self):
        """Cargador mas cercano por ruta, prefiriendo los que se alcanzan con reserva."""
        opciones = []
        for c in ESTACIONES_CARGA:
            d = self.distancia(None, c)
            if d is not None:
                opciones.append((d, c))
        if not opciones:
            return None
        opciones.sort()
        for d, c in opciones:
            if self.pct is None or self.pct >= K_PCT_PER_M * d + RESERVA_PCT:
                return c
        d, c = opciones[0]                # ninguno con reserva: el mas cercano si llega
        if self.pct >= K_PCT_PER_M * d:
            return c
        return None

    # ---- acciones
    def navegar(self, key):
        """Lleva el robot a una estacion. Devuelve True al llegar."""
        nom, x, y, yaw = ESTACIONES[key]
        fallos = 0
        while rclpy.ok() and fallos <= REINTENTOS_NAV:
            self.esperar_si_detenido()
            goal = crear_pose(x, y, yaw, desde=(self.robot['x'], self.robot['y']))
            self.destino(nom)
            self.estado(f'NAVEGANDO_A_{nom.upper().replace(" ", "_")}')
            self.ubicacion = None
            self.nav.goToPose(goal)

            llegada = interrumpido = False
            while not self.nav.isTaskComplete():
                if self.estop or self.pausa:
                    self.nav.cancelTask()
                    interrumpido = True
                    break
                fb = self.nav.getFeedback()
                if fb:
                    d = math.hypot(x - fb.current_pose.pose.position.x,
                                   y - fb.current_pose.pose.position.y)
                    if d <= TOLERANCIA_LLEGADA_M:
                        llegada = True
                        self.nav.cancelTask()
                        break
                rclpy.spin_once(self.nav, timeout_sec=0.1)

            if interrumpido:
                self.evento(f'Tramo a {nom} detenido; se reanuda al liberar')
                continue                  # no cuenta como fallo

            if llegada or self.nav.getResult() == TaskResult.SUCCEEDED:
                self.ubicacion = key
                self.evento(f'Llegada a {nom} (bateria {self.texto_bateria()})')
                return True

            fallos += 1
            self.evento(f'Nav2 no pudo llegar a {nom} (intento {fallos})')
            self.spin(2.0)
        return False

    def cargar(self, objetivo):
        objetivo = min(100.0, objetivo)
        self.cargado = False
        self.estado('CARGANDO')
        self.evento(f'Cargando en {nombre(self.ubicacion)} hasta {objetivo:.0f}% '
                    f'(o instruccion "cargado"). Bateria {self.texto_bateria()}')
        while rclpy.ok():
            self.esperar_si_detenido()
            if self.cargado or (self.pct is not None and self.pct >= objetivo):
                break
            self.spin(1.0)
        self.evento(f'Carga terminada. Bateria {self.texto_bateria()}')

    def ir_a(self, key):
        """Va a una estacion revisando bateria; si no alcanza, pasa antes a cargar."""
        sin_ruta = 0
        while rclpy.ok():
            self.esperar_si_detenido()
            self.spin(0.3)
            if self.ubicacion == key:
                return True

            self.estado('PLANEANDO')
            d = self.distancia(None, key)
            if d is None:
                sin_ruta += 1
                self.evento(f'Sin ruta hacia {nombre(key)} (intento {sin_ruta})')
                if sin_ruta >= REINTENTOS_RUTA:
                    return False
                self.spin(3.0)
                continue

            necesaria = K_PCT_PER_M * d + RESERVA_PCT
            detalle = f'{nombre(key)} {d:.2f} m'
            if key not in ESTACIONES_CARGA:
                c, dc = self.cargador_desde(key)
                necesaria += K_PCT_PER_M * dc
                detalle += f' + regreso a {nombre(c)} {dc:.2f} m'

            if necesaria > 100.0:
                self.evento(f'Tramo imposible: requiere {necesaria:.0f}% ({detalle})')
                return False

            if self.pct is None or self.pct >= necesaria:
                self.evento(f'Bateria OK ({self.texto_bateria()} >= {necesaria:.1f}%): {detalle}')
                return self.navegar(key)

            self.evento(f'Bateria insuficiente ({self.texto_bateria()} < {necesaria:.1f}%) '
                        f'para {detalle}')
            objetivo = max(PCT_CARGADO, necesaria + 5.0)

            if self.ubicacion in ESTACIONES_CARGA:
                self.cargar(objetivo)
                continue

            c = self.elegir_cargador()
            if c is None:
                self.estado('SIN_BATERIA')
                self.evento('No alcanza ningun cargador. Lleva el robot a cargar '
                            'y manda la instruccion "cargado".')
                self.cargado = False
                while rclpy.ok() and not self.cargado:
                    self.spin(1.0)
                continue

            self.evento(f'Desvio a cargar en {nombre(c)}')
            if self.navegar(c):
                self.cargar(objetivo)
            # se vuelve a evaluar el tramo pendiente desde donde quedo el robot

        return False

    def tramo(self, key):
        """Insiste en llegar a una estacion; si falla, espera 'reanudar' de FT Optix."""
        while rclpy.ok() and not self.ir_a(key):
            self.estado('ERROR')
            self.evento(f'No se pudo llegar a {nombre(key)}. '
                        f'Revisa el robot y manda la instruccion "reanudar".')
            self.pausa = True
            self.esperar_si_detenido()

    def esperar_pedido(self):
        self.estado('ESPERANDO_PEDIDO')
        self.destino('')
        self.evento(f'En Inicio: esperando pedido por MQTT hasta {T_ESPERA_PEDIDO:.0f} s')
        fin = time.time() + T_ESPERA_PEDIDO
        while rclpy.ok() and time.time() < fin:
            if self.estop or self.pausa:
                restante = fin - time.time()
                self.esperar_si_detenido()
                fin = time.time() + restante    # la pausa no consume el tiempo de espera
            if self.pedido is not None:
                obj, self.pedido = self.pedido, None
                return obj, 'MQTT'
            self.spin(0.2)
        return random.choice(sorted(OBJETO_A_ALMACEN)), 'ALEATORIO'

    # ---- ciclo principal
    def correr(self):
        self.esperar_nav2()
        self.evento('Mision autonoma iniciada: AGV Pool -> Inicio')
        self.tramo(INICIO)
        while rclpy.ok():
            obj, origen = self.esperar_pedido()
            alm = OBJETO_A_ALMACEN[obj]
            self.objeto(obj, origen)
            self.evento(f'Objeto {obj} ({origen}) -> {nombre(alm)}')

            self.tramo(alm)
            if not rclpy.ok():
                break

            self.estado('DESCARGANDO')
            self.spin(T_DESCARGA)
            self.entregas += 1
            self.pub_entregas.publish(Int32(data=self.entregas))
            self.evento(f'Entrega {self.entregas} completada: objeto {obj} en {nombre(alm)}')
            self.objeto(0, '')

            self.tramo(INICIO)


def main():
    rclpy.init()
    m = Mision()
    try:
        m.correr()
    except KeyboardInterrupt:
        print('\nInterrupcion: cancelando tarea actual...', flush=True)
        try:
            m.nav.cancelTask()
        except Exception:
            pass
    finally:
        m.nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
