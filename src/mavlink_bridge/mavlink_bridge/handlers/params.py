"""Parámetros: MP los pide al conectar y puede leerlos/escribirlos.
   Se puede hacer desde MP pero si lo dejamos en el código ya nos 
   queda cada vez que re-flasheemos la placa"""
from mavlink_bridge.mav_link import mavlink

def _nombre(x):
    return (x.decode() if isinstance(x, bytes) else x).rstrip('\x00')

class Params:
    # Tabla inicial: después la movemos a un YAML. Los nombres siguen los de ArduRover
    # para que MP los muestre en su lugar habitual.
    DEFAULTS = {
        'SYSID_THISMAV': 1.0,
        'FRAME_CLASS': 2.0,      # 2 = barco
        'WP_SPEED': 1.0,         # m/s
        'WP_RADIUS': 2.0,        # m
        'CRUISE_SPEED': 1.0,     # m/s
    }
    POR_TICK = 3                 # cuántos PARAM_VALUE por ciclo (no saturar la radio)

    def __init__(self, link, sysid):
        self.link, self.sysid = link, sysid
        self.valores = dict(self.DEFAULTS)
        self.nombres = list(self.valores)
        self.cola = []

    def handle(self, m, log):
        t = m.get_type()
        if t not in ('PARAM_REQUEST_LIST', 'PARAM_REQUEST_READ', 'PARAM_SET'):
            return False
        if m.target_system not in (0, self.sysid):
            return True
        if t == 'PARAM_REQUEST_LIST':
            self.cola = list(range(len(self.nombres)))
            log.append(f'MP pidió lista de parámetros ({len(self.nombres)})')
        elif t == 'PARAM_REQUEST_READ':
            i = m.param_index
            if i < 0 and _nombre(m.param_id) in self.valores:
                i = self.nombres.index(_nombre(m.param_id))
            if 0 <= i < len(self.nombres):
                self._enviar(i)
        elif t == 'PARAM_SET':
            n = _nombre(m.param_id)
            if n in self.valores:
                self.valores[n] = float(m.param_value)
                self._enviar(self.nombres.index(n))     # eco, como hace ArduPilot
                log.append(f'PARAM_SET {n}={m.param_value}')
        return True

    def tick(self):
        for _ in range(min(self.POR_TICK, len(self.cola))):
            self._enviar(self.cola.pop(0))

    def _enviar(self, i):
        n = self.nombres[i]
        try:
            self.link.mav.param_value_send(
                n.encode(), self.valores[n],
                mavlink.MAV_PARAM_TYPE_REAL32, len(self.nombres), i)
        except OSError:
            pass