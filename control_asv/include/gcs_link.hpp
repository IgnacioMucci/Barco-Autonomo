// =============================================================================
// gcs_link.hpp
//
// Clase que hace de "vehiculo falso" ante Mission Planner:
//   - abre un socket UDP,
//   - manda un HEARTBEAT por segundo (asi Mission Planner cree que hay un barco),
//   - lee lo que manda Mission Planner y lo decodifica,
//   - responde lo minimo que Mission Planner pide al conectarse
//     (parametros, lista de mision) para que no se quede esperando,
//   - permite mandarle textos (STATUSTEXT) que aparecen en su pantalla.
//
// NO usa ROS. Solo sockets de Linux + los headers de MAVLink (c_library_v2).
// Asi se puede probar sola y despues el nodo ROS 2 solo la "envuelve".
// =============================================================================
#pragma once

#include <arpa/inet.h>   // inet_pton, sockaddr_in
#include <fcntl.h>       // fcntl (socket no bloqueante)
#include <netdb.h>       // getaddrinfo (resolver nombres como host.docker.internal)
#include <netinet/in.h>
#include <sys/socket.h>  // socket, bind, sendto, recvfrom
#include <unistd.h>      // close
#include <cerrno>
#include <cstring>
#include <functional>
#include <set>
#include <string>

// Headers de MAVLink (carpeta third_party/c_library_v2). Define todos los
// mensajes del dialecto "common" (HEARTBEAT, STATUSTEXT, COMMAND_LONG, etc.).
#include "common/mavlink.h"

class GcsLink {
 public:
  // Funcion para escribir logs (el nodo ROS le pasa RCLCPP_INFO).
  using LogFn = std::function<void(const std::string &)>;
  // Funcion que se llama cada vez que Mission Planner manda un COMMAND_LONG.
  using CommandFn = std::function<void(const mavlink_command_long_t &)>;

  CommandFn on_command;  // la completa el nodo ROS

  GcsLink(uint8_t sysid, uint8_t compid, uint16_t listen_port, LogFn log)
      : sysid_(sysid), compid_(compid), listen_port_(listen_port), log_(log) {}

  ~GcsLink() {
    if (fd_ >= 0) close(fd_);
  }

  // Destino inicial opcional (modo "UDP" de Mission Planner: la companion
  // empieza a mandar a la IP del ground). Si no se llama, la companion espera
  // a que Mission Planner le escriba primero (modo "UDPCI") y le contesta a
  // quien le escribio.
  bool set_gcs(const std::string &host, uint16_t port) {
    // getaddrinfo acepta tanto una IP ("192.168.1.10") como un nombre
    // ("host.docker.internal", "localhost").
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo *res = nullptr;
    if (getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 || res == nullptr) return false;
    gcs_addr_ = *reinterpret_cast<sockaddr_in *>(res->ai_addr);
    gcs_addr_.sin_port = htons(port);
    freeaddrinfo(res);
    has_gcs_ = true;
    return true;
  }

  // Crea el socket UDP y lo "ata" (bind) al puerto de escucha.
  bool open() {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    // Permite reiniciar el nodo sin esperar a que el SO libere el puerto.
    int yes = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);  // escucha en todas las interfaces
    local.sin_port = htons(listen_port_);
    if (bind(fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) return false;

    // No bloqueante: recvfrom() devuelve enseguida si no hay datos.
    // Asi poll() puede llamarse desde un timer de ROS sin trabar el nodo.
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    return true;
  }

  bool has_gcs() const { return has_gcs_; }
  bool armed() const { return armed_; }

  // ---------------------------------------------------------------------------
  // RECEPCION: leer todos los paquetes pendientes y procesarlos.
  // Llamar seguido (el nodo lo hace cada 10 ms).
  // ---------------------------------------------------------------------------
  void poll() {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    while (true) {
      sockaddr_in from{};
      socklen_t from_len = sizeof(from);
      ssize_t n = recvfrom(fd_, buf, sizeof(buf), 0,
                           reinterpret_cast<sockaddr *>(&from), &from_len);
      if (n <= 0) break;  // no hay mas datos pendientes

      // Un datagrama UDP trae uno o mas mensajes MAVLink. Se los pasa byte a
      // byte al parser, que avisa (devuelve 1) cuando completo un mensaje valido
      // (valida cabecera y checksum).
      for (ssize_t i = 0; i < n; ++i) {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
          // Recordar a quien contestarle: el ultimo que nos escribio.
          if (!has_gcs_ || gcs_addr_.sin_addr.s_addr != from.sin_addr.s_addr ||
              gcs_addr_.sin_port != from.sin_port) {
            gcs_addr_ = from;
            has_gcs_ = true;
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
            log_(std::string("Destino GCS: ") + ip + ":" + std::to_string(ntohs(from.sin_port)));
          }
          handle_message(msg);
        }
      }
    }
  }

  // ---------------------------------------------------------------------------
  // ENVIO
  // ---------------------------------------------------------------------------

  // HEARTBEAT: "estoy vivo, soy un barco con piloto ArduPilot". 1 Hz.
  // Si no llega, Mission Planner da la conexion por caida.
  void send_heartbeat() {
    mavlink_heartbeat_t hb{};
    hb.type = MAV_TYPE_SURFACE_BOAT;          // Mission Planner lo muestra como Rover
    hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (armed_) hb.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    hb.custom_mode = 0;                       // 0 = Manual en Rover
    hb.system_status = armed_ ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode(sysid_, compid_, &msg, &hb);
    send(msg);
  }

  // STATUSTEXT: texto que Mission Planner muestra en el HUD y en la pestaña
  // "Messages". MAVLink limita el texto a 50 caracteres: se corta el resto.
  void send_statustext(uint8_t severity, const std::string &text) {
    mavlink_statustext_t st{};
    st.severity = severity;
    std::strncpy(st.text, text.c_str(), sizeof(st.text) - 1);
    mavlink_message_t msg;
    mavlink_msg_statustext_encode(sysid_, compid_, &msg, &st);
    send(msg);
  }

 private:
  // Serializa un mensaje MAVLink y lo manda por UDP al GCS.
  void send(const mavlink_message_t &msg) {
    if (!has_gcs_) return;  // todavia no sabemos a quien mandarle
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&gcs_addr_), sizeof(gcs_addr_));
  }

  // Decide que hacer con cada mensaje que llega de Mission Planner.
  void handle_message(const mavlink_message_t &msg) {
    // Log de cada tipo de mensaje la primera vez que aparece (para ver el trafico).
    if (seen_.insert(msg.msgid).second) {
      log_("Primer mensaje msgid=" + std::to_string(msg.msgid) +
           " desde sysid=" + std::to_string(msg.sysid) +
           " compid=" + std::to_string(msg.compid));
    }

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        // Mission Planner pide TODOS los parametros al conectarse y espera la
        // respuesta. Contestamos con un unico parametro para que termine rapido.
        mavlink_param_value_t pv{};
        std::strncpy(pv.param_id, "SYSID_THISMAV", sizeof(pv.param_id));
        pv.param_value = sysid_;
        pv.param_type = MAV_PARAM_TYPE_REAL32;
        pv.param_count = 1;
        pv.param_index = 0;
        mavlink_message_t out;
        mavlink_msg_param_value_encode(sysid_, compid_, &out, &pv);
        send(out);
        break;
      }
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
        // Mission Planner pregunta cuantos waypoints hay cargados. Respondemos 0.
        mavlink_mission_request_list_t req;
        mavlink_msg_mission_request_list_decode(&msg, &req);
        mavlink_mission_count_t mc{};
        mc.target_system = msg.sysid;
        mc.target_component = msg.compid;
        mc.count = 0;
        mc.mission_type = req.mission_type;
        mavlink_message_t out;
        mavlink_msg_mission_count_encode(sysid_, compid_, &out, &mc);
        send(out);
        break;
      }
      case MAVLINK_MSG_ID_COMMAND_LONG: {
        // Comandos desde Mission Planner (botones de la pestaña Actions).
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);

        // Solo aceptamos ARM/DISARM (comando 400) como prueba; el resto se
        // rechaza como "no soportado" (esto incluye los pedidos automaticos
        // de Mission Planner, ej. AUTOPILOT_VERSION).
        uint8_t result = MAV_RESULT_UNSUPPORTED;
        if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
          armed_ = (cmd.param1 > 0.5f);
          result = MAV_RESULT_ACCEPTED;
        }

        // COMMAND_ACK: la respuesta que Mission Planner espera tras un comando.
        mavlink_command_ack_t ack{};
        ack.command = cmd.command;
        ack.result = result;
        ack.target_system = msg.sysid;
        ack.target_component = msg.compid;
        mavlink_message_t out;
        mavlink_msg_command_ack_encode(sysid_, compid_, &out, &ack);
        send(out);

        if (on_command) on_command(cmd);  // avisar al nodo ROS
        break;
      }
      default:
        break;  // el resto de los mensajes se ignora
    }
  }

  uint8_t sysid_, compid_;
  uint16_t listen_port_;
  LogFn log_;
  int fd_ = -1;
  sockaddr_in gcs_addr_{};
  bool has_gcs_ = false;
  bool armed_ = false;
  std::set<uint32_t> seen_;
};