import json
from queue import Queue, Empty, Full

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.set_message import set_message_fields

BROKER = '192.168.50.15'   # mismo broker que ros_to_mqtt.py

# Mensajes de FT Optix (formato {"Timestamp": ..., "Records": [{"TagName": alias, "Value": v}]})
# MQTT topic: (ROS topic, message type, field alias)
#   nexo/task      -> /task      Int32   objeto 1, 2 o 3 (pedido para mision_autonoma)
#   nexo/instruct  -> /instruct  String  'pausa' | 'reanudar' | 'cargado'
#   nexo/estop     -> /estop     Bool    true = paro de emergencia
# Para recibir algo nuevo de FT Optix basta con agregar una linea aqui.
MQTT_TOPICS = {
    'nexo/task': ('/task', 'std_msgs/msg/Int32', 'task'),
    'nexo/instruct': ('/instruct', 'std_msgs/msg/String', 'instruct'),
    'nexo/estop': ('/estop', 'std_msgs/msg/Bool', 'estop'),
}


def reconstruct(records, alias):
    data = {}

    for record in records:
        tag = record['TagName']
        value = record['Value']

        if not isinstance(tag, str) or not tag:
            raise ValueError('Invalid TagName')

        if tag == alias:
            tag = 'data'

        fields = tag.split('-')
        if any(not field for field in fields):
            raise ValueError('Invalid field path')

        current = data

        for field in fields[:-1]:
            if field not in current:
                current[field] = {}
            if not isinstance(current[field], dict):
                raise ValueError('Conflicting field path')
            current = current[field]

        if fields[-1] in current:
            raise ValueError('Duplicate field')

        current[fields[-1]] = value

    return data


def validar(msg_type, value):
    """Revisa y convierte el valor segun el tipo de mensaje de ROS."""
    tipo = msg_type.split('/')[-1]
    if tipo == 'Bool':
        if isinstance(value, str) and value.strip().lower() in ('true', 'false', '1', '0'):
            return value.strip().lower() in ('true', '1')
        if type(value) is bool or value in (0, 1):
            return bool(value)
        raise ValueError('Invalid Bool')
    if tipo in ('Int32', 'Int64', 'UInt16'):
        if isinstance(value, float) and value.is_integer():
            value = int(value)
        if isinstance(value, str) and value.strip().lstrip('-').isdigit():
            value = int(value.strip())
        if type(value) is not int or not -(2**31) <= value < 2**31:
            raise ValueError(f'Invalid {tipo}')
        return value
    if tipo in ('Float32', 'Float64'):
        if type(value) in (int, float):
            return float(value)
        raise ValueError(f'Invalid {tipo}')
    if tipo == 'String':
        if not isinstance(value, str):
            raise ValueError('Invalid String')
        return value
    return value


class MQTTToROS(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros')

        self.inbox = Queue(maxsize=100)
        self.publishers_map = {}
        self.types = {}

        for topic, (ros_topic, msg_type, _) in MQTT_TOPICS.items():
            self.types[topic] = get_message(msg_type)
            self.publishers_map[topic] = self.create_publisher(
                self.types[topic], ros_topic, 10
            )

        self.mqtt = mqtt.Client(client_id='nexo_ros_in')
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.mqtt.connect(BROKER, 1883)
        self.mqtt.loop_start()

        self.timer = self.create_timer(0.02, self.process_messages)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            for topic in MQTT_TOPICS:
                client.subscribe(topic, qos=1)

    def on_message(self, client, userdata, message):
        try:
            self.inbox.put_nowait((message.topic, message.payload))
        except Full:
            self.get_logger().warning('MQTT queue full')

    def process_messages(self):
        for _ in range(100):
            try:
                topic, raw = self.inbox.get_nowait()
            except Empty:
                break

            try:
                payload = json.loads(raw.decode('utf-8'))

                if not isinstance(payload, dict):
                    raise ValueError('Expected JSON object')
                if not isinstance(payload.get('Timestamp'), str):
                    raise ValueError('Missing Timestamp')
                if not isinstance(payload.get('Records'), list):
                    raise ValueError('Missing Records array')

                _, _, alias = MQTT_TOPICS[topic]
                fields = reconstruct(payload['Records'], alias)

                if set(fields) != {'data'}:
                    raise ValueError('Expected one data field')

                value = fields['data']
                _, msg_type, _ = MQTT_TOPICS[topic]
                value = validar(msg_type, value)
                fields['data'] = value

                ros_msg = self.types[topic]()
                set_message_fields(ros_msg, fields)
                self.publishers_map[topic].publish(ros_msg)
                self.get_logger().info(f'MQTT {topic} -> {MQTT_TOPICS[topic][0]}: {value!r}')

            except (ValueError, TypeError, KeyError,
                    AttributeError, AssertionError,
                    UnicodeDecodeError) as exc:
                self.get_logger().warning(
                    f'Invalid message on {topic}: {exc}'
                )

    def destroy_node(self):
        self.mqtt.disconnect()
        self.mqtt.loop_stop()
        return super().destroy_node()


def main():
    rclpy.init()
    node = MQTTToROS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
