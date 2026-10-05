# QUEDÓ VIEJO
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil


class NodoHeartbeatGround(Node):
    def __init__(self):
        super().__init__('nodo_heartbeat_ground')
        self.declare_parameter('puerto', '/dev/ttyUSB3')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('sysid', 2)  # distinto al del Pixhawk (1)

        self.m = mavutil.mavlink_connection(
            self.get_parameter('puerto').value,
            baud=self.get_parameter('baud').value,
            source_system=self.get_parameter('sysid').value,
            source_component=1)

        self.n = 0
        self.create_timer(1.0, self.enviar)

    def enviar(self):
        mav = mavutil.mavlink
        self.m.mav.heartbeat_send(
            mav.MAV_TYPE_GROUND_ROVER,
            mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0, 0, mav.MAV_STATE_STANDBY)

        self.n += 1
        txt = f"Hola desde companion #{self.n}"
        self.m.mav.statustext_send(mav.MAV_SEVERITY_INFO, txt.encode())
        self.get_logger().info(f"Enviado: {txt}")


def main():
    rclpy.init()
    node = NodoHeartbeatGround()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()