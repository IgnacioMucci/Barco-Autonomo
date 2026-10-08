"""Capa MAVLink pura (sin ROS) para hablar con Mission Planner por UDP."""
import os
os.environ.setdefault("MAVLINK20", "1")  # forzar MAVLink 2 (antes de importar mavutil)

from pymavlink import mavutil  # noqa: E402

mavlink = mavutil.mavlink


class AsvLink:
    def __init__(self, gcs_ip, gcs_port=14550, sysid=1, compid=1):
        self.sysid = sysid
        self.conn = mavutil.mavlink_connection(
            f'udpout:{gcs_ip}:{gcs_port}',
            source_system=sysid, source_component=compid)
        self.mav = self.conn.mav

    def heartbeat(self):
        self.mav.heartbeat_send(
            mavlink.MAV_TYPE_SURFACE_BOAT,
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0,
            mavlink.MAV_STATE_ACTIVE)

    def status(self, text, severity=mavlink.MAV_SEVERITY_INFO):
        # STATUSTEXT: max 50 bytes. Mission Planner lo muestra en el HUD
        self.mav.statustext_send(severity, text.encode()[:50])

    def poll(self):
        """Procesa lo recibido, responde y devuelve lineas de log."""
        log = []
        while True:
            try:
                m = self.conn.recv_match(blocking=False)
            except OSError:  # ICMP unreachable si MP aun no escucha
                break
            if m is None:
                break
            t = m.get_type()
            if t == 'BAD_DATA':
                continue
            if t == 'HEARTBEAT':
                log.append(f'RX HEARTBEAT de sys={m.get_srcSystem()} '
                           f'comp={m.get_srcComponent()} type={m.type}')
            elif t == 'PARAM_REQUEST_LIST':
                # MP pide parametros al conectar: devolvemos 1 dummy
                self.mav.param_value_send(
                    b'ASV_DUMMY', 0.0, mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
                log.append('RX PARAM_REQUEST_LIST -> TX PARAM_VALUE dummy')
            elif t == 'MISSION_REQUEST_LIST':
                self.mav.mission_count_send(
                    m.get_srcSystem(), m.get_srcComponent(), 0)
                log.append('RX MISSION_REQUEST_LIST -> TX MISSION_COUNT 0')
            elif t == 'COMMAND_LONG' and m.target_system in (0, self.sysid):
                name = mavlink.enums['MAV_CMD'].get(m.command)
                name = name.name if name else str(m.command)
                self.mav.command_ack_send(
                    m.command, mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                    m.get_srcSystem(), m.get_srcComponent())
                self.status(f'ASV: recibi {name}')
                log.append(f'RX COMMAND_LONG {name} p1={m.param1} '
                           f'-> TX COMMAND_ACK + STATUSTEXT')
        return log
