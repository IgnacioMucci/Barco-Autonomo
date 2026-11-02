"""Estado del vehículo que la companion le presenta a Mission Planner."""
import math
import time
from dataclasses import dataclass, field
from enum import IntEnum

class RoverMode(IntEnum):
    # Números de modo de ArduRover (custom_mode). En MP se usa para mostrar el nombre.
    MANUAL = 0
    ACRO = 1
    STEERING = 3
    HOLD = 4
    LOITER = 5
    AUTO = 10
    RTL = 11
    SMART_RTL = 12
    GUIDED = 15
    INITIALIZING = 16

# Modos que aceptamos por ahora (el resto se rechaza)
MODOS_SOPORTADOS = {RoverMode.MANUAL, RoverMode.HOLD, RoverMode.AUTO, RoverMode.GUIDED}
# Equivalen a has_manual_input() / is_autopilot_mode() de ArduRover
MODOS_MANUALES = {RoverMode.MANUAL, RoverMode.ACRO, RoverMode.STEERING}
MODOS_AUTOPILOTO = {RoverMode.AUTO, RoverMode.GUIDED, RoverMode.RTL, RoverMode.SMART_RTL}

@dataclass
class VehicleState:
    armed: bool = False
    mode: int = int(RoverMode.HOLD)
    failsafe: bool = False
    # posición (en el paso siguiente la llena el GPS RTK desde ROS)
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    fix_type: int = 0          # 0 sin fix, 3 = 3D, 5 = RTK float, 6 = RTK fixed
    sats: int = 0
    # actitud [rad] y velocidad
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    groundspeed: float = 0.0   # m/s
    cog_deg: float = 0.0       # rumbo de movimiento
    throttle: int = 0          # %
    # home: ArduRover lo fija al armar si todavía no hay uno
    home_set: bool = False
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0
    t0: float = field(default_factory=time.monotonic)

    @property
    def heading_deg(self):
        return math.degrees(self.yaw) % 360.0

    def boot_ms(self):
        return int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF