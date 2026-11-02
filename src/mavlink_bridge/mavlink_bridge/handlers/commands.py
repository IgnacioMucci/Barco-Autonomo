"""Comandos de MP: armar, cambiar modo, pedir versión."""
from mavlink_bridge.mav_link import mavlink
from mavlink_bridge.vehicle_state import MODOS_SOPORTADOS, RoverMode

class Commands:
    # ahora también recibimos telemetría
    def __init__(self, link, state, sysid, telemetry):
        self.link, self.state, self.sysid = link, state, sysid
        self.telemetry = telemetry

    def handle(self, m, log):
        t = m.get_type()
        if t in ('COMMAND_LONG', 'COMMAND_INT'):
            if m.target_system in (0, self.sysid):
                self._comando(m, log)
            return True
        if t == 'SET_MODE':                         # forma vieja de cambiar de modo
            if m.target_system == self.sysid:
                self._cambiar_modo(int(m.custom_mode), log)
            return True
        if t == 'AUTOPILOT_VERSION_REQUEST':        # se solicita versión de autopilot
            if m.target_system in (0, self.sysid):
                self._autopilot_version()
            return True
        if t in ('REQUEST_DATA_STREAM', 'STATUSTEXT'):
            return True      # los reconocemos para no llenar el log
        return False

    def _ack(self, m, resultado):
        self.link.mav.command_ack_send(
            m.command, resultado, 0, 0, m.get_srcSystem(), m.get_srcComponent())

    def _cambiar_modo(self, modo, log):
        if modo not in MODOS_SOPORTADOS:
            log.append(f'modo {modo} no soportado')
            return False
        self.state.mode = modo
        log.append(f'modo -> {RoverMode(modo).name}')
        self.link.status(f'Modo {RoverMode(modo).name}')
        return True

    def _comando(self, m, log):
        c, ok, no = m.command, mavlink.MAV_RESULT_ACCEPTED, mavlink.MAV_RESULT_UNSUPPORTED

        # modificado el armado para que se fije y se publique el home
        if c == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:   
            self.state.armed = m.param1 >= 0.5
            if self.state.armed:
                self._fijar_home_si_falta()
            log.append('ARMADO' if self.state.armed else 'DESARMADO')
            self.link.status('Armado' if self.state.armed else 'Desarmado')
            self._ack(m, ok)
            if self.state.armed:
                self._enviar_home()        

        elif c == mavlink.MAV_CMD_DO_SET_MODE:    # param2 = custom_mode
            fallo = mavlink.MAV_RESULT_FAILED
            self._ack(m, ok if self._cambiar_modo(int(m.param2), log) else fallo)

        elif c == mavlink.MAV_CMD_MISSION_START:  # ArduRover lo trata como "pasar a AUTO"
            self._cambiar_modo(int(RoverMode.AUTO), log)
            self._ack(m, ok)

        elif c in (mavlink.MAV_CMD_REQUEST_MESSAGE,
                   mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES):
            pedido = int(m.param1) if c == mavlink.MAV_CMD_REQUEST_MESSAGE else 148
            if pedido == mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                self._ack(m, ok)
                self._autopilot_version()
            elif pedido == mavlink.MAVLINK_MSG_ID_HOME_POSITION and self.state.home_set:    
                self._ack(m, ok)    # agregar opción de setear home
                self._enviar_home()
            else:
                self._ack(m, no)

        elif c == mavlink.MAV_CMD_GET_HOME_POSITION:    # get home
            if self.state.home_set:
                self._ack(m, ok)
                self._enviar_home()
            else:
                self._ack(m, mavlink.MAV_RESULT_FAILED)

        elif c == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:         # setear intervalo de mensaje
            ok_int = self.telemetry.set_interval(int(m.param1), int(m.param2))
            self._ack(m, ok if ok_int else no)

        else:   # cualquier otra cosa: decirlo explícitamente evita que MP se quede esperando
            log.append(f'comando {c} no soportado')
            self._ack(m, no)

    def _autopilot_version(self):
        caps = (mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                mavlink.MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2)
        # 0x040500FF = "ArduRover 4.5.0 release": solo para que MP active su UI estándar
        self.link.mav.autopilot_version_send(
            caps, 0x040500FF, 0, 0, 0, [0] * 8, [0] * 8, [0] * 8, 0, 0, 0)
    
    def _fijar_home_si_falta(self):
        s = self.state
        if not s.home_set:
            s.home_lat, s.home_lon, s.home_alt = s.lat, s.lon, s.alt_m
            s.home_set = True

    def _enviar_home(self):
        s = self.state
        self.link.mav.home_position_send(
            int(s.home_lat * 1e7), int(s.home_lon * 1e7), int(s.home_alt * 1000),
            0.0, 0.0, 0.0,              # x, y, z locales
            [1.0, 0.0, 0.0, 0.0],       # cuaternión
            0.0, 0.0, 0.0,              # approach
            0)                          # time_usec