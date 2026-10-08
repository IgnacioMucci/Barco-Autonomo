""" Este archivo es el que hace de traducción hacia ros2
Ahora podemos establecer la conexión mediante udp o puerto serie 
según el texto que enviemos en el parámetro "conexión" por comando:
# UDP
ros2 run mavlink_bridge ground_link_node --ros-args -p conexion:=udpout:<IP_GROUND>:14550
# Radio USB
ros2 run mavlink_bridge ground_link_node --ros-args -p conexion:=/dev/ttyUSB0 -p baud:=57600
"""

import rclpy                  # libreria cliente de ROS 2 para Python, ROS Client Library for Python.
from rclpy.node import Node   # clase base de todo nodo

from mavlink_bridge.mav_link import AsvLink  # nuestra capa MAVLink

class HeartbeatNode(Node):    # mi clase heartbeatnode hereda de Node
    # constructor
    def __init__(self):
        super().__init__('mavlink_ground_link')  # nombre del nodo que creamos en el grafo de ROS /mavlink_ground_link XX/mavlink_heartbeat

        # Parámetros de ROS para conexiones usb o udp, se pueden cambiar al lanzar con --ros-args -p nombre:=valor
        self.declare_parameter('conexion', '/dev/ttyUSB3')  # puerto serie, o 'udpout:IP:14550'
        self.declare_parameter('baud', 57600)               # solo se usa en serie
        self.declare_parameter('sysid', 1)

        """ Parametros de ROS viejos, para conexiones udp: se pueden cambiar al lanzar con --ros-args -p nombre:=valor
        self.declare_parameter('gcs_ip', '192.168.0.229')   # IP de la PC con Mission Planner
        self.declare_parameter('gcs_port', 14550)           # puerto UDP donde escucha MP
        self.declare_parameter('sysid', 1)                  # ID MAVLink de este "vehiculo"""

        # Leemos el valor final de cada parametro (el default o el se pasó por consola)
        conexion = self.get_parameter('conexion').value     
        baud = self.get_parameter('baud').value             
        sysid = self.get_parameter('sysid').value

        """Getters viejos de conexión udp - Leo el valor final de cada parametro (el default o el que pasaste por consola)
        ip = self.get_parameter('gcs_ip').value
        port = self.get_parameter('gcs_port').value
        sysid = self.get_parameter('sysid').value"""

        # acá creamos la capa mavlink con ros2
        self.link = AsvLink(conexion, baud, sysid)          # abro la conexion (serie o UDP segun el texto)
        self.n = 0                                          # contador de mensajes de texto enviados
        self.get_logger().info(f'MAVLink -> {conexion} @ {baud} (sysid {sysid})')   # log de arranque

        """ Links viejos con udp
        self.link = AsvLink(ip, port, sysid)  # abro el socket UDP hacia MP
        self.n = 0                            # contador de mensajes de texto enviados
        self.get_logger().info(f'MAVLink -> udp {ip}:{port} (sysid {sysid})')  # log de arranque"""

        # Timers de ROS: cada uno llama a su funcion periodicamente
        self.create_timer(1.0, self.on_heartbeat)   # 1 Hz, cada 1 segundo ejecutamos on_heartbeat
        self.create_timer(5.0, self.on_status)      # mensaje breve cada 5 s de status
        self.create_timer(0.05, self.on_rx)         # leer respuestas de MP (polling) cada 50ms

    def on_heartbeat(self):
        self.link.heartbeat()  # manda el HEARTBEAT (MP lo necesita ~1 vez por segundo)

    def on_status(self):
        self.n += 1
        self.link.status(f'ASV hola desde ROS2 #{self.n}')  # mensaje de texto al HUD de MP

    def on_rx(self):
        # Procesa lo que mando MP (poll ya contesta) y lo imprime en la terminal de ROS
        # hace un for de la lista de elementos que devuelve el poll()
        for line in self.link.poll():
            self.get_logger().info(line)


def main():
    rclpy.init()             # inicia ROS 2
    node = HeartbeatNode()   # crea el nodo (arranca los timers)
    try:
        rclpy.spin(node)     # queda corriendo y ejecutando los timers hasta Ctrl+C
    except KeyboardInterrupt:
        pass                 # Ctrl+C: salida normal
    node.destroy_node()      # libera el nodo
    rclpy.try_shutdown()     # apaga ROS 2 (sin error si ya estaba apagado)


if __name__ == '__main__':
    main()
