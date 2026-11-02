"""Nodo ROS 2 fino: toda la lógica MAVLink vive en gcs_server.py.

# UDP
ros2 run mavlink_bridge ground_link_node --ros-args -p conexion:=udpout:<IP_GROUND>:14550
# Radio USB
ros2 run mavlink_bridge ground_link_node --ros-args -p conexion:=/dev/ttyUSB0 -p baud:=57600"""

import rclpy                  # libreria cliente de ROS 2 para Python, ROS Client Library for Python.
from rclpy.node import Node   # clase base de todo nodo

from mavlink_bridge.gcs_server import GcsServer # nuestra capa MAVLink


class GroundLinkNode(Node):
    def __init__(self):
        # nombre del nodo que creamos en el grafo de ROS /mavlink_ground_link
        super().__init__('mavlink_ground_link')             
        self.declare_parameter('conexion', '/dev/ttyUSB3')  # puerto serie, o 'udpout:IP:14550'
        self.declare_parameter('baud', 57600)               # solo se usa en serie
        self.declare_parameter('sysid', 1)
        # Solo para probar el HUD/mapa sin GPS: poné lat/lon distintos de 0
        self.declare_parameter('fake_lat', 0.0)
        self.declare_parameter('fake_lon', 0.0)

        # Leemos el valor final de cada parametro (el default o el q se pasó por consola)
        conexion = self.get_parameter('conexion').value
        baud = self.get_parameter('baud').value
        sysid = self.get_parameter('sysid').value
        
        # acá creamos la capa mavlink con ros2
        # abro la conexión de la computadora companion con la placa de autonavegación
        # y con la computadora ground (serie o UDP segun el texto que mandemos por consola)
        self.server = GcsServer(conexion, baud, sysid)
        self.get_logger().info(f'MAVLink -> {conexion} @ {baud} (sysid {sysid})')   # log de arranque

        lat = self.get_parameter('fake_lat').value
        lon = self.get_parameter('fake_lon').value
        if lat or lon:
            s = self.server.state
            s.lat, s.lon, s.fix_type, s.sats = lat, lon, 3, 10

        self.create_timer(0.02, self.on_tick)   # 50 Hz, cada 20ms se ejecuta on_tick

    def on_tick(self):
        for line in self.server.step():
            self.get_logger().info(line)            


def main():
    rclpy.init()            # inicia ROS 2
    node = GroundLinkNode() # crea el nodo (arranca los timers)
    try:
        rclpy.spin(node)    # queda corriendo y ejecutando los timers hasta Ctrl+C
    except KeyboardInterrupt:
        pass                 # Ctrl+C: salida normal
    node.destroy_node()      # libera el nodo
    rclpy.try_shutdown()     # apaga ROS 2 (sin error si ya estaba apagado)


if __name__ == '__main__':
    main()
