"""Capa MAVLink pura (sin ROS) para hablar con Mission Planner por UDP."""
import os
# MAVLINK20=1 hace que pymavlink use MAVLink 2 (sin esto usa MAVLink 1 y a algunos
# mensajes les faltan campos). Tiene que setearse ANTES de importar mavutil.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402  (import despues del setenv a proposito)

# Atajo: ahi estan las constantes (MAV_TYPE_*, MAV_STATE_*...) y las definiciones de mensajes
mavlink = mavutil.mavlink


class AsvLink:
    """ Este era el constructor para cuando se usaban udp para la comunicación
    def __init__(self, gcs_ip, gcs_port=14550, sysid=1, compid=1):
        # Guardo mi ID de sistema para despues filtrar los comandos dirigidos a mi
        self.sysid = sysid
        # 'udpout:IP:PUERTO' = yo inicio el envio hacia esa IP y puerto (Mission Planner escucha ahi).
        # source_system / source_component = con que ID me identifico en cada paquete (1/1 = vehiculo)
        self.conn = mavutil.mavlink_connection(
            f'udpout:{gcs_ip}:{gcs_port}',
            source_system=sysid, source_component=compid)
        # Objeto con los metodos *_send() para mandar cada tipo de mensaje
        self.mav = self.conn.mav"""

    # Constructor actualizado, la conexión puede ser por udp o por puerto serie
    def __init__(self, conexion, baud=57600, sysid=1, compid=1):
        # Guardo mi ID de sistema para despues filtrar los comandos dirigidos a mi
        self.sysid = sysid
        # 'conexion' es un texto y pymavlink decide el medio segun cómo empiece:
        #   'udpout:IP:PUERTO' -> UDP;  '/dev/ttyUSB0' -> puerto serie (usa baud)
        # cuestión, abrimos la conexión con los parámetros correctos
        self.conn = mavutil.mavlink_connection(
            conexion, baud=baud,
            source_system=sysid, source_component=compid)
        # Objeto con los metodos *_send() para mandar cada tipo de mensaje
        self.mav = self.conn.mav      

    # función que manda un mensaje mavlink de tipo heartbeat
    def heartbeat(self):
        self.mav.heartbeat_send(
            mavlink.MAV_TYPE_SURFACE_BOAT,               # tipo de vehiculo: barco de superficie
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,         # me presento como autopilot tipo ArduPilot
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   # base_mode: flag "modo custom habilitado"
            0,                                           # custom_mode: numero de modo propio (0 = ninguno)
            mavlink.MAV_STATE_ACTIVE)                    # estado del sistema: activo

    # función que manda un mensaje mavlink de tipo status
    def status(self, text, severity=mavlink.MAV_SEVERITY_INFO):
        # STATUSTEXT: max 50 bytes. Mission Planner lo muestra en el HUD
        # encode() hace str -> bytes, recortado a 50 bytes (el limite del mensaje STATUSTEXT)
        # esto lo hacemos porque mavlink transmite bytes
        self.mav.statustext_send(severity, text.encode()[:50])

    # función que hace polling, preguntando si mission planner mandó algo (y procesa los msj pendientes)
    def poll(self):
        # Procesa lo recibido, responde y devuelve lineas de log.
        log = []  # lineas de texto que el nodo ROS va a imprimir
        while True:  # vacía todo lo que haya llegado desde la última vez
            try:
                # pregunta a pymavlink si hay msj mavlink recibido
                m = self.conn.recv_match(blocking=False)
                # blocking=False: si no hay nada, devuelve m=None en vez de quedarse esperando
            except OSError:  # ICMP unreachable si MP aun no escucha
                break
            if m is None:  # no queda nada pendiente: salgo del while
                break
            t = m.get_type()  # nombre del mensaje, p.ej. 'HEARTBEAT'
            if t == 'BAD_DATA':  # bytes que no se pudieron decodificar: los ignoro
                continue
            if t == 'HEARTBEAT':
                # Heartbeat de la ground: get_srcSystem/Component = quien lo mando (MP = 255)
                log.append(f'RX HEARTBEAT de sys={m.get_srcSystem()} '
                           f'comp={m.get_srcComponent()} type={m.type}')
            elif t == 'PARAM_REQUEST_LIST':
                # MP pide parametros al conectar: devolvemos 1 dummy
                # (id, valor, tipo float32, total de parametros = 1, indice = 0)
                self.mav.param_value_send(
                    b'ASV_DUMMY', 0.0, mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
                log.append('RX PARAM_REQUEST_LIST -> TX PARAM_VALUE dummy')
            elif t == 'MISSION_REQUEST_LIST':
                # MP pide la lista de waypoints: respondo "0 items" al que pregunto
                self.mav.mission_count_send(
                    m.get_srcSystem(), m.get_srcComponent(), 0)
                log.append('RX MISSION_REQUEST_LIST -> TX MISSION_COUNT 0')
            # COMMAND_LONG: comandos de MP (arm, cambio de modo, etc.). Solo si va para mi (o a todos: 0)
            elif t == 'COMMAND_LONG' and m.target_system in (0, self.sysid):
                name = mavlink.enums['MAV_CMD'].get(m.command)  # numero de comando -> entrada del enum
                name = name.name if name else str(m.command)    # nombre legible; si no existe, el numero
                # COMMAND_ACK: confirmo que recibi el comando (ACCEPTED), dirigido a quien lo envio
                self.mav.command_ack_send(
                    m.command, mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                    m.get_srcSystem(), m.get_srcComponent())
                self.status(f'ASV: recibi {name}')  # ademas aviso por texto al HUD de MP que recibí el comando y lo acepté
                log.append(f'RX COMMAND_LONG {name} p1={m.param1} '
                           f'-> TX COMMAND_ACK + STATUSTEXT')
        return log
