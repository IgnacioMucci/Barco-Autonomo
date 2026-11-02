"""Une todos los handlers: esto es lo que antes hacía la Pixhawk frente a Mission Planner."""
import time

from mavlink_bridge.handlers.commands import Commands
from mavlink_bridge.handlers.mission import Mission
from mavlink_bridge.handlers.params import Params
from mavlink_bridge.handlers.telemetry import Telemetry
from mavlink_bridge.mav_link import AsvLink, mavlink
from mavlink_bridge.vehicle_state import VehicleState

GCS_TIMEOUT = 5.0   # segundos sin heartbeat de MP => enlace perdido

class GcsServer:
    def __init__(self, conexion, baud=57600, sysid=1):
        self.link = AsvLink(conexion, baud, sysid)
        self.state = VehicleState()
        self.telemetry = Telemetry(self.link, self.state)
        self.params = Params(self.link, sysid)
        self.mission = Mission(self.link, sysid)        
        self.commands = Commands(self.link, self.state, sysid, self.telemetry)  #add: telemetrías
        self._handlers = (self.params, self.mission, self.commands)
        self._ultimo_hb_gcs = None
        self._gcs_ok = False
        self._vistos = set()

    def step(self):
        # Un ciclo: atiende lo recibido y manda la telemetría que toque. Devuelve líneas de log.
        log = []
        for m in self.link.recibir():
            t = m.get_type()
            if t == 'HEARTBEAT':
                if m.type == mavlink.MAV_TYPE_GCS:
                    self._ultimo_hb_gcs = time.monotonic()
                continue
            # cada handler devuelve True si reconoció el mensaje
            if not any([h.handle(m, log) for h in self._handlers]):
                if t not in self._vistos:        # solo la primera vez, para no hacer spam
                    self._vistos.add(t)
                    log.append(f'mensaje sin handler: {t}')
        self.params.tick()
        self.telemetry.tick()
        self._vigilar_gcs(log)
        return log

    def _vigilar_gcs(self, log):
        hb = self._ultimo_hb_gcs
        vivo = hb is not None and time.monotonic() - hb < GCS_TIMEOUT
        if vivo and not self._gcs_ok:
            log.append('GCS conectada')
        elif not vivo and self._gcs_ok:
            log.append('GCS PERDIDA (sin heartbeat)')
            # Acá va después el failsafe (ej. pasar a HOLD), como FS_GCS_ENABLE de ArduRover
        self._gcs_ok = vivo