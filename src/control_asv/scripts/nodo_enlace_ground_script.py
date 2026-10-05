#!/usr/bin/env python3
"""
nodo_enlace_ground.py  -  Enlace companion <-> ground usando MAVROS como puente a la radio.

MAVROS tiene que correr con gcs_url apuntando a la radio (ver enlace_ground_launch.py).
Este nodo NO abre el puerto serie: solo habla ROS 2 con MAVROS.

COMPANION -> GROUND  (lo ve Mission Planner)
  /mavros/statustext/send   mavros_msgs/StatusText  -> STATUSTEXT        (pestaña Mensajes)
  /mavros/debug_value/send  mavros_msgs/DebugValue  -> NAMED_VALUE_FLOAT (Ctrl+F > MAVLink Inspector)

GROUND -> COMPANION
  /uas1/mavlink_source      mavros_msgs/Mavlink     <- todo lo que llega por la radio
  Se decodifica y se republica como topicos ROS 2 normales:
  /asv/ground/texto         std_msgs/String
  /asv/ground/valor         mavros_msgs/DebugValue

Por que /uas1/mavlink_source y no /mavros/statustext/recv ni /mavros/debug_value/*:
los plugins de MAVROS descartan todo mensaje cuyo sysid no sea el del vehiculo (1),
y ground manda con sysid 255. El topico crudo no filtra nada.
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)

from mavros_msgs.msg import Mavlink, StatusText, DebugValue
from std_msgs.msg import String

# IDs de mensaje MAVLink (common.xml)
MSG_HEARTBEAT = 0
MSG_NAMED_VALUE_FLOAT = 251
MSG_NAMED_VALUE_INT = 252
MSG_STATUSTEXT = 253


def _cstr(b: bytes) -> str:
    """bytes terminados en \\0 -> str."""
    return b.split(b'\0', 1)[0].decode('utf-8', errors='replace')


def decodificar(msgid: int, payload: bytes):
    """Devuelve un dict con los campos que nos interesan, o None si no es un mensaje que usemos.

    El payload de MAVLink2 viene sin los ceros finales, por eso se rellena a 255 bytes.
    Offsets verificados contra pymavlink (orden de campos en el cable).
    """
    p = payload.ljust(255, b'\0')
    if msgid == MSG_HEARTBEAT:
        return {'tipo': 'HEARTBEAT', 'mav_type': p[4], 'autopilot': p[5]}
    if msgid == MSG_STATUSTEXT:
        return {'tipo': 'STATUSTEXT', 'severidad': p[0], 'texto': _cstr(p[1:51])}
    if msgid == MSG_NAMED_VALUE_FLOAT:
        return {'tipo': 'NAMED_VALUE_FLOAT', 'nombre': _cstr(p[8:18]),
                'valor': struct.unpack_from('<f', p, 4)[0]}
    if msgid == MSG_NAMED_VALUE_INT:
        return {'tipo': 'NAMED_VALUE_INT', 'nombre': _cstr(p[8:18]),
                'valor': struct.unpack_from('<i', p, 4)[0]}
    return None


class NodoEnlaceGround(Node):
    def __init__(self):
        super().__init__('nodo_enlace_ground')

        self.declare_parameter('periodo', 1.0)   # s entre envios
        self.declare_parameter('uas', 'uas1')    # prefijo del bus MAVLink de MAVROS (/uas<tgt_system>)
        self.declare_parameter('tgt_system', 1)  # sysid del vehiculo; todo lo que NO sea este viene de afuera

        periodo = self.get_parameter('periodo').value
        uas = self.get_parameter('uas').value
        self.tgt_system = self.get_parameter('tgt_system').value

        # ---- TX: companion -> ground ----
        # statustext/send de MAVROS escucha con SensorDataQoS; debug_value/send con QoS(10) reliable.
        self.pub_texto = self.create_publisher(
            StatusText, '/mavros/statustext/send', qos_profile_sensor_data)
        self.pub_valor = self.create_publisher(
            DebugValue, '/mavros/debug_value/send', 10)

        # ---- RX: ground -> companion ----
        qos_raw = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Mavlink, f'/{uas}/mavlink_source', self.mavlink_cb, qos_raw)
        self.pub_g_texto = self.create_publisher(String, '/asv/ground/texto', 10)
        self.pub_g_valor = self.create_publisher(DebugValue, '/asv/ground/valor', 10)

        self.n = 0
        self.create_timer(periodo, self.tick)
        self.get_logger().info(
            f'Enlace listo. TX: /mavros/statustext/send + /mavros/debug_value/send | '
            f'RX: /{uas}/mavlink_source (sysid != {self.tgt_system})')

    # ------------------------------------------------------------------ TX
    def enviar_texto(self, texto: str, severidad: int = StatusText.INFO):
        """STATUSTEXT a ground. MAVLink limita a 50 caracteres."""
        m = StatusText()
        m.header.stamp = self.get_clock().now().to_msg()
        m.severity = severidad
        m.text = texto[:50]
        self.pub_texto.publish(m)

    def enviar_float(self, nombre: str, valor: float):
        """NAMED_VALUE_FLOAT a ground. El nombre tiene max. 10 caracteres."""
        m = DebugValue()
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
        m.index = -1
        m.array_id = -1
        m.name = nombre[:10]
        m.value_float = float(valor)
        self.pub_valor.publish(m)

    def enviar_int(self, nombre: str, valor: int):
        """NAMED_VALUE_INT a ground."""
        m = DebugValue()
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = DebugValue.TYPE_NAMED_VALUE_INT
        m.index = -1
        m.array_id = -1
        m.name = nombre[:10]
        m.value_int = int(valor)
        self.pub_valor.publish(m)

    def tick(self):
        self.n += 1
        # Diagnostico: si nadie escucha, MAVROS no esta corriendo (o el plugin debug_value esta en denylist)
        if self.pub_texto.get_subscription_count() == 0 or self.pub_valor.get_subscription_count() == 0:
            self.get_logger().warn(
                'MAVROS no esta escuchando /mavros/statustext/send o /mavros/debug_value/send',
                throttle_duration_sec=5.0)
        self.enviar_texto(f'Hola desde companion #{self.n}')
        self.enviar_float('ASV_CNT', self.n)
        self.get_logger().info(f'TX -> ground: "Hola desde companion #{self.n}" | ASV_CNT={self.n}')

    # ------------------------------------------------------------------ RX
    def mavlink_cb(self, m: Mavlink):
        if m.framing_status != Mavlink.FRAMING_OK or m.sysid == self.tgt_system:
            return  # paquete roto, o del vehiculo/MAVROS mismo

        payload = b''.join(struct.pack('<Q', w) for w in m.payload64)[:m.len]
        d = decodificar(m.msgid, payload)
        if d is None:
            return
        origen = f'{m.sysid}.{m.compid}'

        if d['tipo'] == 'HEARTBEAT':
            self.get_logger().info(
                f'RX <- ground [{origen}] HEARTBEAT (mav_type={d["mav_type"]})',
                throttle_duration_sec=5.0)
        elif d['tipo'] == 'STATUSTEXT':
            self.get_logger().info(f'RX <- ground [{origen}] TEXTO: "{d["texto"]}"')
            self.pub_g_texto.publish(String(data=d['texto']))
        else:
            self.get_logger().info(f'RX <- ground [{origen}] {d["nombre"]} = {d["valor"]}')
            v = DebugValue()
            v.header.stamp = self.get_clock().now().to_msg()
            v.index = -1
            v.array_id = -1
            v.name = d['nombre']
            if d['tipo'] == 'NAMED_VALUE_FLOAT':
                v.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
                v.value_float = float(d['valor'])
            else:
                v.type = DebugValue.TYPE_NAMED_VALUE_INT
                v.value_int = int(d['valor'])
            self.pub_g_valor.publish(v)


def main():
    rclpy.init()
    nodo = NodoEnlaceGround()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
