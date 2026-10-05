#!/usr/bin/env python3
# Corre en la PC de ground (pip install pymavlink). Cerrar Mission Planner antes si usa el mismo COM.
#   python ground_test.py COM5          (Windows)    |    python ground_test.py /dev/ttyUSB0
import sys, time
from pymavlink import mavutil

conn = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
m = mavutil.mavlink_connection(conn, baud=57600, source_system=255, source_component=190)
n = 0
t0 = time.time()
while True:
    n += 1
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    m.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, f"Hola desde ground #{n}".encode())
    m.mav.named_value_float_send(int((time.time() - t0) * 1000) & 0xFFFFFFFF, b'GND_CNT', float(n))
    print(f"TX -> companion: #{n}")
    t_end = time.time() + 1.0
    while time.time() < t_end:   # mientras espera, imprime lo que llega del companion
        r = m.recv_match(type=['STATUSTEXT', 'NAMED_VALUE_FLOAT'], blocking=True, timeout=0.2)
        if r is not None:
            print(f"RX <- [{r.get_srcSystem()}.{r.get_srcComponent()}]", r)