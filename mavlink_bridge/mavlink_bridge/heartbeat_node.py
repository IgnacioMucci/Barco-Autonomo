import rclpy
from rclpy.node import Node

from mavlink_bridge.mav_link import AsvLink

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('mavlink_heartbeat')
        self.declare_parameter('gcs_ip', '192.168.0.135')  # IP de la PC con Mission Planner
        self.declare_parameter('gcs_port', 14550)
        self.declare_parameter('sysid', 1)

        ip = self.get_parameter('gcs_ip').value
        port = self.get_parameter('gcs_port').value
        sysid = self.get_parameter('sysid').value

        self.link = AsvLink(ip, port, sysid)
        self.n = 0
        self.get_logger().info(f'MAVLink -> udp {ip}:{port} (sysid {sysid})')

        self.create_timer(1.0, self.on_heartbeat)   # 1 Hz
        self.create_timer(5.0, self.on_status)      # mensaje breve cada 5 s
        self.create_timer(0.05, self.on_rx)         # leer respuestas de MP

    def on_heartbeat(self):
        self.link.heartbeat()

    def on_status(self):
        self.n += 1
        self.link.status(f'ASV hola desde ROS2 #{self.n}')

    def on_rx(self):
        for line in self.link.poll():
            self.get_logger().info(line)


def main():
    rclpy.init()
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
