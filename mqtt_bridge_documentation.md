# Módulo de Comunicación MQTT / ROS 2 (`nexo_mqtt`)

Este documento detalla la arquitectura, el flujo de datos, la configuración de red y los procedimientos de ejecución para el paquete **`nexo_mqtt`**, encargado del enlace en tiempo real entre la pila de control local del robot (ROS 2 Humble) y el servidor Broker MQTT central del laboratorio.

---

## 1. Visión General de la Arquitectura

El sistema utiliza dos nodos principales para mantener la telemetría continua y la recepción de comandos de alto nivel:

1. **`ros_to_mqtt` (Telemetría de salida):** Captura el estado operativo del robot desde los tópicos locales de ROS 2, calcula transformaciones trigonométricas (conversión de cuaterniones a grados sexagesimales $\text{Yaw}$), estructura la información bajo el formato JSON estandarizado y la transmite al Broker MQTT.
2. **`mqtt_to_ros` (Comandos de entrada):** Escucha instrucciones externas, paros de emergencia (`/estop`) y tareas deseadas desde el Broker MQTT y los descompone hacia tópicos nativos de ROS 2.

```
 +------------------------+        ROS 2 Topics       +-------------------+        MQTT Network       +----------------------+
 | battery_manager        |  --> /nexo/battery/* -->  |                   |                           |                      |
 | location_manager       |  --> /nexo/current_pose ->|    ros_to_mqtt    |  --> wlan1 (192.168.50.x) |   Broker MQTT        |
 | smac_navigator         |  --> /nexo/events/*  -->  |                   |      (Port 1883)          |   192.168.50.15      |
 +------------------------+                           +-------------------+                           +----------------------+
```

---

## 2. Configuración de Red e Interfaz Inalámbrica

Para resolver la comunicación entre la subred del control local y el servidor broker, la Raspberry Pi / MentorPi utiliza una **antena Wi-Fi USB dedicada (`wlan1`)**.

* **IP del Broker MQTT:** `192.168.50.15` (Puerto `1883`)
* **Interfaz de salida asignada:** `wlan1`
* **Dirección IP del robot en `wlan1`:** `192.168.50.20`
* **Prefijo Tópicos MQTT:** `nexo/`

> **Nota de Enrutamiento:** El nodo `ros_to_mqtt` fuerza el socket TCP/IP utilizando el parámetro `bind_address='192.168.50.20'`, evitando que el sistema desvíe paquetes por el adaptador inalámbrico integrado (`wlan0`).

---

## 3. Especificación del Formato de Datos JSON

Cada paquete transmitido por el nodo `ros_to_mqtt` incluye un sello de tiempo en formato ISO 8601 UTC y un arreglo plano de pares etiqueta-valor (`Records`):

### A. Tópico Pose Actual (`/nexo/current_pose` -> `nexo/nexo/current_pose`)
Extrae las posiciones planas ($X, Y$) y convierte la orientación representada en cuaterniones ($z, w$) al ángulo de yaw sexagesimal:

$$\text{Yaw} = \operatorname{atan2}\left(2(w \cdot z), 1 - 2(z^2)\right) \times \frac{180}{\pi}$$

**Ejemplo de Payload JSON:**
```json
{
  "Timestamp": "2026-11-06T21:38:00.000Z",
  "Records": [
    { "TagName": "position-x", "Value": 1.452 },
    { "TagName": "position-y", "Value": -0.321 },
    { "TagName": "orientation-yaw_deg", "Value": 45.12 }
  ]
}
```

### B. Tópico Estado de Batería (`/nexo/battery/percentage` -> `nexo/nexo/battery/percentage`)
```json
{
  "Timestamp": "2026-11-06T21:38:00.000Z",
  "Records": [
    { "TagName": "data", "Value": 88.5 }
  ]
}
```

---

## 4. Código Fuente Implementado (`ros_to_mqtt.py`)

```python
from datetime import datetime, timezone
import json
import math
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

BROKER = '192.168.50.15'
PREFIX = 'nexo/'

# Mapeo de tópicos ROS 2 a monitorear
ROS_TOPICS = {
    '/nexo/battery/percentage': 'std_msgs/msg/Float32',
    '/nexo/battery/status': 'std_msgs/msg/String',
    '/nexo/current_pose': 'geometry_msgs/msg/PoseStamped',
    '/nexo/events/arrival': 'std_msgs/msg/String',
}


def quaternion_to_yaw_deg(q):
    """Calcula el angulo yaw en grados sexagesimales a partir del cuaternion."""
    siny_cosp = 2.0 * (q.get('w', 1.0) * q.get('z', 0.0))
    cosy_cosp = 1.0 - 2.0 * (q.get('z', 0.0) ** 2)
    return round(math.degrees(math.atan2(siny_cosp, cosy_cosp)), 2)


def flatten(data, prefix=''):
    records = []
    for key, value in data.items():
        tag = f'{prefix}-{key}' if prefix else key
        if isinstance(value, dict):
            records.extend(flatten(value, tag))
        else:
            records.append({'TagName': tag, 'Value': value})
    return records


class ROSToMQTT(Node):

    def __init__(self):
        super().__init__('ros_to_mqtt')

        self.mqtt = mqtt.Client(client_id='nexo_ros_out')
        self.mqtt.on_connect = self.on_mqtt_connect

        self.get_logger().info(f'Conectando al Broker MQTT en {BROKER}:1883...')
        # Se fuerza el amarre a la interfaz de la antena USB (wlan1)
        self.mqtt.connect(BROKER, 1883, bind_address='192.168.50.20')
        self.mqtt.loop_start()

        self.subscriptions_list = []
        for topic, msg_type in ROS_TOPICS.items():
            sub = self.create_subscription(
                get_message(msg_type),
                topic,
                lambda msg, t=topic: self.forward(msg, t),
                10,
            )
            self.subscriptions_list.append(sub)

        self.get_logger().info(
            '======================================================\n'
            ' [OK] Nodo ROS_to_MQTT iniciado correctamente.\n'
            ' Escuchando bateria, pose y eventos para reenviar a MQTT.\n'
            '======================================================'
        )

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(' Conexión establecida exitosamente con el broker MQTT.')
        else:
            self.get_logger().error(f' Fallo al conectar con el broker MQTT. Codigo de error: {rc}')

    def forward(self, msg, topic):
        timestamp = (
            datetime.now(timezone.utc)
            .isoformat(timespec='milliseconds')
            .replace('+00:00', 'Z')
        )

        if topic == '/nexo/current_pose':
            pos = msg.pose.position
            ori = msg.pose.orientation
            yaw_deg = quaternion_to_yaw_deg(
                {'x': ori.x, 'y': ori.y, 'z': ori.z, 'w': ori.w}
            )

            records = [
                {'TagName': 'position-x', 'Value': round(pos.x, 3)},
                {'TagName': 'position-y', 'Value': round(pos.y, 3)},
                {'TagName': 'orientation-yaw_deg', 'Value': yaw_deg},
            ]
        else:
            dict_msg = message_to_ordereddict(msg)
            records = flatten(dict_msg)

        payload = {'Timestamp': timestamp, 'Records': records}

        mqtt_topic = PREFIX + topic.lstrip('/')
        self.mqtt.publish(mqtt_topic, json.dumps(payload), qos=1)
        self.get_logger().info(f'Dato retransmitido a MQTT -> {mqtt_topic}')

    def destroy_node(self):
        self.mqtt.disconnect()
        self.mqtt.loop_stop()
        return super().destroy_node()


def main():
    rclpy.init()
    node = ROSToMQTT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

---

## 5. Instrucciones de Compilación y Ejecución

### Paso 1: Compilar el paquete de ROS 2

```zsh
source /opt/ros/humble/setup.zsh
cd ~/shared/nexo_ws
colcon build --packages-select nexo_mqtt
source install/setup.zsh
```

### Paso 2: Ejecutar el nodo `ros_to_mqtt`

En la pestaña designada para el puente MQTT de salida:

```zsh
source /opt/ros/humble/setup.zsh
cd ~/shared/nexo_ws
source install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 run nexo_mqtt ros_to_mqtt
```

### Paso 3: Monitorización en tiempo real

Para comprobar desde cualquier terminal de la red los datos recibidos por el Broker MQTT:

```zsh
mosquitto_sub -h 192.168.50.15 -t "nexo/#" -v
```