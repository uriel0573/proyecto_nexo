from datetime import datetime, timezone
import json
import math
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

BROKER = '192.168.50.15'
PREFIX = ''

# Mapeo topicos de ROS del proyecto
ROS_TOPICS = {
    '/nexo/battery/percentage': 'std_msgs/msg/Float32',
    '/nexo/battery/status': 'std_msgs/msg/String',
    '/nexo/battery/voltage': 'std_msgs/msg/Float32',
    '/nexo/battery/autonomy_min': 'std_msgs/msg/Float32',
    '/nexo/telemetry/speed': 'std_msgs/msg/Float32',
    '/nexo/telemetry/distance': 'std_msgs/msg/Float32',
    '/nexo/current_pose': 'geometry_msgs/msg/PoseStamped',
    '/nexo/events/arrival': 'std_msgs/msg/String',
    # Mision autonoma (mision_autonoma.py)
    '/nexo/mission/state': 'std_msgs/msg/String',
    '/nexo/mission/object': 'std_msgs/msg/Int32',
    '/nexo/mission/destination': 'std_msgs/msg/String',
    '/nexo/mission/order_source': 'std_msgs/msg/String',
    '/nexo/mission/deliveries': 'std_msgs/msg/Int32',
    '/nexo/mission/event': 'std_msgs/msg/String',
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

        # Configuracion del cliente mqtt con callbacks
        self.mqtt = mqtt.Client(client_id='nexo_ros_out')
        self.mqtt.on_connect = self.on_mqtt_connect

        self.get_logger().info(f'Conectando al Broker MQTT en {BROKER}:1883...')
        self.mqtt.connect(BROKER, 1883, bind_address='192.168.50.20')
        self.mqtt.loop_start()

        # Crear suscriptores a los topicos de ros
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
            ' Escuchando bateria, voltaje, autonomia, velocidad,\n'
            ' distancia, pose y eventos para reenviar a MQTT.\n'
            '======================================================'
        )

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Conexion establecida exitosamente con el broker MQTT.')
        else:
            self.get_logger().error(f'Fallo al conectar con el broker MQTT. Codigo de error: {rc}')

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
        self.get_logger().debug(f'Dato retransmitido a MQTT -> {mqtt_topic}')

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
