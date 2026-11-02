"""Protocolo de misión. Por ahora: home como ítem 0 y misión vacía (la subida viene después)."""
from mavlink_bridge.mav_link import mavlink

class Mission:
    def __init__(self, link, state, sysid):
        self.link, self.state, self.sysid = link, state, sysid
        self.items = []          # acá se guardan los waypoints reales (sin el home)

    def handle(self, m, log):
        t = m.get_type()
        mav = self.link.mav
        src = (m.get_srcSystem(), m.get_srcComponent())
        tipo = getattr(m, 'mission_type', 0)      # 0=misión, 1=geovalla, 2=puntos de rally

        if t == 'MISSION_REQUEST_LIST':
            n = len(self.items) + 1 if tipo == 0 else 0     # +1 por el home
            mav.mission_count_send(*src, n, mission_type=tipo)
            log.append(f'MISSION_REQUEST_LIST -> MISSION_COUNT {n}')
            return True

        if t in ('MISSION_REQUEST_INT', 'MISSION_REQUEST'):
            if tipo != 0 or m.seq != 0:
                # los waypoints reales los implementamos en el próximo paso
                mav.mission_ack_send(*src, mavlink.MAV_MISSION_INVALID_SEQUENCE,
                                     mission_type=tipo)
                log.append(f'{t} seq={m.seq}: no disponible')
                return True
            s = self.state
            if t == 'MISSION_REQUEST_INT':
                mav.mission_item_int_send(
                    *src, 0, mavlink.MAV_FRAME_GLOBAL, mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1, 0, 0, 0, 0,
                    int(s.home_lat * 1e7), int(s.home_lon * 1e7), s.home_alt,
                    mission_type=0)
            else:   # versión float
                mav.mission_item_send(
                    *src, 0, mavlink.MAV_FRAME_GLOBAL, mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1, 0, 0, 0, 0,
                    s.home_lat, s.home_lon, s.home_alt, mission_type=0)
            log.append('home enviado como ítem 0')
            return True

        if t == 'MISSION_CLEAR_ALL':
            if tipo == 0:
                self.items.clear()
            mav.mission_ack_send(*src, mavlink.MAV_MISSION_ACCEPTED, mission_type=tipo)
            return True
        return False