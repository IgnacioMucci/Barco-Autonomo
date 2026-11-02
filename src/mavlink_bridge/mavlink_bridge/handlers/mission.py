"""Protocolo de misión. Por ahora solo contesta 'no hay misión' para que MP no espere."""
from mavlink_bridge.mav_link import mavlink

class Mission:
    def __init__(self, link, sysid):
        self.link, self.sysid = link, sysid
        self.items = []          # en el próximo paso: acá se guardan los waypoints subidos

    def handle(self, m, log):
        t = m.get_type()
        if t == 'MISSION_REQUEST_LIST':
            tipo = getattr(m, 'mission_type', 0)      # 0=misión, 1=geovalla, 2=puntos rally
            n = len(self.items) if tipo == 0 else 0
            self.link.mav.mission_count_send(
                m.get_srcSystem(), m.get_srcComponent(), n, mission_type=tipo)
            log.append(f'MISSION_REQUEST_LIST -> MISSION_COUNT {n}')
            return True
        if t == 'MISSION_CLEAR_ALL':
            tipo = getattr(m, 'mission_type', 0)
            if tipo == 0:
                self.items.clear()
            self.link.mav.mission_ack_send(
                m.get_srcSystem(), m.get_srcComponent(),
                mavlink.MAV_MISSION_ACCEPTED, mission_type=tipo)
            return True
        return False