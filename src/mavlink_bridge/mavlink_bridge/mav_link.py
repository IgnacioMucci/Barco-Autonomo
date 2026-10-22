"""Capa MAVLink pura (sin ROS) para hablar con Mission Planner por UDP."""
import os
# MAVLINK20=1 hace que pymavlink use MAVLink 2 (sin esto usa MAVLink 1 y a algunos
# mensajes les faltan campos). Tiene que setearse ANTES de importar mavutil.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402  (import despues del setenv a proposito)

# Atajo: ahi estan las constantes (MAV_TYPE_*, MAV_STATE_*...) y las definiciones de mensajes
mavlink = mavutil.mavlink

class AsvLink:

    # Constructor actualizado, la conexión puede ser por udp o por puerto serie
    # Incluimos los parámetros de comunicación con la placa pixhawk (fcu)
    def __init__(self, conexion, baud=57600, sysid=1, compid=1,
                fcu='/dev/ttyACM0', fcu_baud=115200):
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
        # Segunda conexión: la Pixhawk por USB
        self.fc = mavutil.mavlink_connection(
            fcu, baud=fcu_baud,
            source_system=sysid, source_component=compid) 

    # función que manda un mensaje mavlink de tipo heartbeat
    def heartbeat(self):
        self.mav.heartbeat_send(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER,         # tipo de vehiculo: computadora companion
            mavlink.MAV_AUTOPILOT_INVALID,               # NO me presento como autopilot, porque eso lo hace la placa 
            0, 0,                                        # custom_mode: numero de modo propio (0 = ninguno)
            mavlink.MAV_STATE_ACTIVE)                    # estado del sistema: activo

    # función que manda un mensaje mavlink de tipo status
    def status(self, text, severity=mavlink.MAV_SEVERITY_INFO):
        # STATUSTEXT: max 50 bytes. Mission Planner lo muestra en el HUD
        # encode() hace str -> bytes, recortado a 50 bytes (el limite del mensaje STATUSTEXT)
        # esto lo hacemos porque mavlink transmite bytes
        self.mav.statustext_send(severity, text.encode()[:50])


    # Tipos que vale la pena mostrar en el log (el resto es telemetría y haría spam)
    LOG_TIPOS = ('COMMAND_LONG', 'COMMAND_INT', 'COMMAND_ACK', 'SET_MODE',
                 'PARAM_SET', 'MISSION_COUNT', 'MISSION_REQUEST',
                 'MISSION_REQUEST_INT', 'MISSION_ITEM', 'MISSION_ITEM_INT',
                 'MISSION_ACK')

    # función privada para reenviar
    def _reenviar(self, origen, destino, etiqueta, log):
        # Lee todo lo pendiente en 'origen' y lo escribe tal cual en 'destino'
        while True:
            try:
                m = origen.recv_match(blocking=False)
                if m is None:
                    break
                t = m.get_type()
                if t == 'BAD_DATA':
                    continue
                destino.write(m.get_msgbuf())   # bytes originales, sin re-empaquetar
            except OSError:
                break
            # if t in self.LOG_TIPOS:
            #     log.append(f'{etiqueta} {t}')

            # log más completo para verificar que todos los heartbeats estén siendo reconocidos
            if t in self.LOG_TIPOS:
                extra = f' src={m.get_srcSystem()}/{m.get_srcComponent()}'
                if t in ('COMMAND_LONG', 'COMMAND_INT'):
                    extra += f' cmd={m.command}'
                elif t == 'COMMAND_ACK':
                    extra += f' cmd={m.command} result={m.result}'
                # el campo este de result me sirve para debuggear qué está pasando con los mensajes:
                # En result, 0 es aceptado, 2 denegado, 3 no soportado, 4 falló y 5 en progreso.
                elif t == 'MISSION_COUNT':
                    extra += f' count={m.count} tgt={m.target_system}/{m.target_component}'
                elif t in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
                    extra += f' seq={m.seq} tgt={m.target_system}/{m.target_component}'
                # FC pide siempre seq=0 (no acepta el ítem) o pide seq=1 y MP no reacciona
                elif t in ('MISSION_ITEM', 'MISSION_ITEM_INT'):
                    extra += f' seq={m.seq} cmd={m.command} frame={m.frame} tgt={m.target_system}/{m.target_component}'
                elif t == 'MISSION_ACK':
                    extra += f' type={m.type}'
                log.append(f'{etiqueta} {t}{extra}')
            

    # función que hace polling, preguntando si mission planner 
    # o la placa mandaron algo (y procesa los msj pendientes)
    def poll(self):
        log = []        # lineas de texto que el nodo ROS va a imprimir
        self._reenviar(self.conn, self.fc, 'GCS->FC', log)
        self._reenviar(self.fc, self.conn, 'FC->GCS', log)
        return log


    # FUNCIÓN VIEJAAAA # función que hace polling, preguntando si mission planner mandó algo (y procesa los msj pendientes)
    # def poll(self):
    #     # Procesa lo recibido, responde y devuelve lineas de log.
    #     log = []  # lineas de texto que el nodo ROS va a imprimir
    #     while True:  # vacía todo lo que haya llegado desde la última vez
    #         try:
    #             # pregunta a pymavlink si hay msj mavlink recibido
    #             m = self.conn.recv_match(blocking=False)
    #             # blocking=False: si no hay nada, devuelve m=None en vez de quedarse esperando
    #         except OSError:  # ICMP unreachable si MP aun no escucha
    #             break
    #         if m is None:  # no queda nada pendiente: salgo del while
    #             break
    #         t = m.get_type()  # nombre del mensaje, p.ej. 'HEARTBEAT'
    #         if t == 'BAD_DATA':  # bytes que no se pudieron decodificar: los ignoro
    #             continue
    #         if t == 'HEARTBEAT':
    #             # Heartbeat de la ground: get_srcSystem/Component = quien lo mando (MP = 255)
    #             log.append(f'RX HEARTBEAT de sys={m.get_srcSystem()} '
    #                        f'comp={m.get_srcComponent()} type={m.type}')
            
    #         """elif t == 'PARAM_REQUEST_LIST':
    #             # MP pide parametros al conectar: devolvemos 1 dummy
    #             # (id, valor, tipo float32, total de parametros = 1, indice = 0)
    #             self.mav.param_value_send(
    #                 b'ASV_DUMMY', 0.0, mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
    #             log.append('RX PARAM_REQUEST_LIST -> TX PARAM_VALUE dummy')
    #         elif t == 'MISSION_REQUEST_LIST':
    #             # MP pide la lista de waypoints: respondo "0 items" al que pregunto
    #             self.mav.mission_count_send(
    #                 m.get_srcSystem(), m.get_srcComponent(), 0)
    #             log.append('RX MISSION_REQUEST_LIST -> TX MISSION_COUNT 0')
    #         # COMMAND_LONG: comandos de MP (arm, cambio de modo, etc.). Solo si va para mi (o a todos: 0)
    #         elif t == 'COMMAND_LONG' and m.target_system in (0, self.sysid):
    #             name = mavlink.enums['MAV_CMD'].get(m.command)  # numero de comando -> entrada del enum
    #             name = name.name if name else str(m.command)    # nombre legible; si no existe, el numero
    #             # COMMAND_ACK: confirmo que recibi el comando (ACCEPTED), dirigido a quien lo envio
    #             self.mav.command_ack_send(
    #                 m.command, mavlink.MAV_RESULT_ACCEPTED, 0, 0,
    #                 m.get_srcSystem(), m.get_srcComponent())
    #             self.status(f'ASV: recibi {name}')  # ademas aviso por texto al HUD de MP que recibí el comando y lo acepté
    #             log.append(f'RX COMMAND_LONG {name} p1={m.param1} '
    #                        f'-> TX COMMAND_ACK + STATUSTEXT')"""
    #     return log
