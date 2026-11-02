"""Capa MAVLink pura (sin ROS): solo el transporte hacia Mission Planner."""
import os
# MAVLINK20=1 hace que pymavlink use MAVLink 2 (sin esto usa MAVLink 1 y a algunos
# mensajes les faltan campos). Tiene que setearse ANTES de importar mavutil.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402  (import despues del set env a proposito)

# Atajo: ahi estan las constantes (MAV_TYPE_*, MAV_STATE_*...) y las definiciones de mensajes
mavlink = mavutil.mavlink

class AsvLink:
    """Conexión con la ground (UDP o serie). Ya no hay conexión con la Pixhawk acá."""

    # Constructor actualizado, la conexión puede ser por udp o por puerto serie
    def __init__(self, conexion, baud=57600, sysid=1, compid=1):
        # Guardo mi ID de sistema para despues filtrar los comandos dirigidos a mi
        self.sysid = sysid
        # 'conexion' es un texto y pymavlink decide el medio segun cómo empiece:
        #   'udpout:IP:PUERTO' -> UDP;  '/dev/ttyUSB0' -> puerto serie (usa baud)
        # cuestión, abrimos la conexión con los parámetros correctos
        self.conn = mavutil.mavlink_connection(
            conexion, baud=baud, source_system=sysid, source_component=compid)
        # Objeto con los metodos *_send() para mandar cada tipo de mensaje
        self.mav = self.conn.mav   # métodos *_send() de cada mensaje
        # ya no hay conexión con placa pixhawk acá, porque no reenviamos nada aun
    
    # función que manda un mensaje mavlink de tipo status        
    def status(self, text, severity=mavlink.MAV_SEVERITY_INFO):
        # STATUSTEXT: max 50 bytes. Mission Planner lo muestra en el HUD
        # encode() hace str -> bytes, recortado a 50 bytes (el limite del mensaje STATUSTEXT)
        # esto lo hacemos porque mavlink transmite bytes        
        self.mav.statustext_send(severity, text.encode()[:50])

    def recibir(self):
        # Devuelve (como generador) todos los mensajes pendientes, sin bloquear.
        while True:
            try:
                m = self.conn.recv_match(blocking=False)
            except OSError:      # por ej. ICMP unreachable si MP todavía no escucha
                return
            if m is None:
                return
            if m.get_type() == 'BAD_DATA':
                continue
            yield m