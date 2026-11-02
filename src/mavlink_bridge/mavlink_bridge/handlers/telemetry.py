"""Telemetría periódica hacia Mission Planner. Por ejemplo acá está el heartbeat que debe emular la companion"""
import math
import time

from mavlink_bridge.mav_link import mavlink
from mavlink_bridge.vehicle_state import MODOS_AUTOPILOTO, MODOS_MANUALES, RoverMode

class Telemetry:
    # período de envío [s] de cada mensaje (acordate de que la radio es de ancho de banda limitado)
    PERIODOS = {'heartbeat': 1.0, 'sys_status': 2.0, 'attitude': 0.1,
                'position': 0.2, 'gps_raw': 0.5, 'vfr_hud': 0.25}
    # id de mensaje MAVLink -> nombre interno
    IDS = {0: 'heartbeat', 1: 'sys_status', 24: 'gps_raw',
           30: 'attitude', 33: 'position', 74: 'vfr_hud'}

    def __init__(self, link, state):
        self.mav, self.state = link.mav, state
        self._ultimo = {n: 0.0 for n in self.PERIODOS}
        self._fn = {'heartbeat': self.heartbeat, 'sys_status': self.sys_status,
                    'attitude': self.attitude, 'position': self.position,
                    'gps_raw': self.gps_raw, 'vfr_hud': self.vfr_hud}

    def tick(self):
        # Llamar seguido: manda lo que ya toca según su período.
        # (cada mensaje de telemetría tiene su frecuencia con la que aparece)
        ahora = time.monotonic()
        for nombre, periodo in self.PERIODOS.items():
            if ahora - self._ultimo[nombre] >= periodo:
                self._ultimo[nombre] = ahora
                try:
                    self._fn[nombre]()
                except OSError:      # el enlace todavía no está listo; reintenta después
                    pass

    def _estado_sistema(self):     # equivale a vehicle_system_status()
        s = self.state
        if s.failsafe:
            return mavlink.MAV_STATE_CRITICAL
        if s.mode == RoverMode.INITIALIZING:
            return mavlink.MAV_STATE_CALIBRATING
        if s.mode == RoverMode.HOLD:
            return mavlink.MAV_STATE_STANDBY
        return mavlink.MAV_STATE_ACTIVE

    def heartbeat(self):           # equivale a base_mode() + custom_mode()
        s = self.state
        base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if s.mode in MODOS_MANUALES:
            base |= mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
        if s.mode in MODOS_AUTOPILOTO:
            base |= mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        if s.armed:
            base |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        self.mav.heartbeat_send(
            mavlink.MAV_TYPE_SURFACE_BOAT,
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,   # ahora SÍ nos presentamos como piloto
            base, s.mode, self._estado_sistema())

    def sys_status(self):
        s = self.state
        gps = mavlink.MAV_SYS_STATUS_SENSOR_GPS
        sano = gps if s.fix_type >= 3 else 0       # solo declaramos lo que realmente tenemos
        self.mav.sys_status_send(
            gps, gps, sano,
            0,           # carga
            65535, -1, -1,   # voltaje / corriente / % batería: desconocidos por ahora
            0, 0, 0, 0, 0, 0)

    def attitude(self):
        s = self.state
        self.mav.attitude_send(s.boot_ms(), s.roll, s.pitch, s.yaw, 0.0, 0.0, 0.0)

    def position(self):
        s = self.state
        cog = math.radians(s.cog_deg)
        self.mav.global_position_int_send(
            s.boot_ms(), int(s.lat * 1e7), int(s.lon * 1e7), int(s.alt_m * 1000), 0,
            int(s.groundspeed * math.cos(cog) * 100),   # vx norte [cm/s]
            int(s.groundspeed * math.sin(cog) * 100),   # vy este  [cm/s]
            0, int(s.heading_deg * 100) % 36000)

    def gps_raw(self):
        s = self.state
        self.mav.gps_raw_int_send(
            s.boot_ms() * 1000, s.fix_type, int(s.lat * 1e7), int(s.lon * 1e7),
            int(s.alt_m * 1000), 65535, 65535,
            int(s.groundspeed * 100), int(s.cog_deg * 100) % 36000, s.sats)

    def vfr_hud(self):
        s = self.state
        self.mav.vfr_hud_send(s.groundspeed, s.groundspeed, int(s.heading_deg),
                              s.throttle, s.alt_m, 0.0)