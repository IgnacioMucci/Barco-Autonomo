// =============================================================================
// gcs_link_node.cpp
//
// Nodo ROS 2 que conecta la computadora ground (Mission Planner) con ROS 2 en
// la companion, SIN Pixhawk. Usa la clase GcsLink (gcs_link.hpp) para hablar
// MAVLink por UDP y la expone como topics:
//
//   /asv/to_gcs    (std_msgs/String)  ROS -> Mission Planner (sale como STATUSTEXT)
//   /asv/from_gcs  (std_msgs/String)  Mission Planner -> ROS (comandos recibidos)
//
// Cualquier otro nodo tuyo en C++ solo tiene que publicar/suscribirse a esos
// dos topics; no necesita saber nada de MAVLink.
// =============================================================================
#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "control_asv/gcs_link.hpp"

using namespace std::chrono_literals;

class GcsLinkNode : public rclcpp::Node {
 public:
  GcsLinkNode() : Node("gcs_link") {
    // ---- Parametros (se pueden cambiar desde el launch) --------------------
    declare_parameter<int>("listen_port", 14555);  // puerto UDP donde escucha la companion
    declare_parameter<std::string>("gcs_ip", "");  // IP o nombre del ground (vacio = esperar a que escriba primero)
    declare_parameter<int>("gcs_port", 14550);     // puerto del ground (solo si gcs_ip no esta vacio)
    declare_parameter<int>("sysid", 1);            // id MAVLink del "barco"
    declare_parameter<int>("compid", 1);           // id de componente (1 = autopiloto)
    declare_parameter<bool>("auto_test", true);    // manda un texto de prueba cada 5 s

    const int listen_port = get_parameter("listen_port").as_int();
    const std::string gcs_ip = get_parameter("gcs_ip").as_string();
    const int gcs_port = get_parameter("gcs_port").as_int();

    // ---- Enlace MAVLink ----------------------------------------------------
    // El ultimo argumento es la funcion de log: escribe en la consola de ROS.
    link_ = std::make_unique<GcsLink>(
        static_cast<uint8_t>(get_parameter("sysid").as_int()),
        static_cast<uint8_t>(get_parameter("compid").as_int()),
        static_cast<uint16_t>(listen_port),
        [this](const std::string &s) { RCLCPP_INFO(get_logger(), "%s", s.c_str()); });

    if (!link_->open()) {
      RCLCPP_FATAL(get_logger(), "No se pudo abrir UDP %d (puerto ocupado?)", listen_port);
      throw std::runtime_error("socket UDP");
    }
    if (!gcs_ip.empty() && !link_->set_gcs(gcs_ip, static_cast<uint16_t>(gcs_port))) {
      RCLCPP_FATAL(get_logger(), "gcs_ip invalida o no se pudo resolver: %s", gcs_ip.c_str());
      throw std::runtime_error("gcs_ip");
    }
    RCLCPP_INFO(get_logger(), "Escuchando MAVLink en UDP %d. Destino inicial: %s",
                listen_port, gcs_ip.empty() ? "(esperando a Mission Planner)" : gcs_ip.c_str());

    // ---- Mission Planner -> ROS -------------------------------------------
    from_gcs_pub_ = create_publisher<std_msgs::msg::String>("/asv/from_gcs", 10);
    link_->on_command = [this](const mavlink_command_long_t &c) {
      // 511/512 son pedidos automaticos de Mission Planner al conectarse
      // (SET_MESSAGE_INTERVAL / REQUEST_MESSAGE); no se publican para no ensuciar.
      if (c.command == 511 || c.command == 512) return;
      std::ostringstream ss;
      ss << "COMMAND_LONG cmd=" << c.command << " p1=" << c.param1 << " p2=" << c.param2
         << " p3=" << c.param3 << " p4=" << c.param4 << " p5=" << c.param5
         << " p6=" << c.param6 << " p7=" << c.param7;
      std_msgs::msg::String out;
      out.data = ss.str();
      from_gcs_pub_->publish(out);
    };

    // ---- ROS -> Mission Planner -------------------------------------------
    to_gcs_sub_ = create_subscription<std_msgs::msg::String>(
        "/asv/to_gcs", 10, [this](const std_msgs::msg::String &m) {
          link_->send_statustext(MAV_SEVERITY_INFO, m.data);  // se trunca a 50 caracteres
        });

    // ---- Timers ------------------------------------------------------------
    // Cada 10 ms: leer lo que llego por UDP.
    poll_timer_ = create_wall_timer(10ms, [this]() { link_->poll(); });
    // Cada 1 s: HEARTBEAT (obligatorio para que Mission Planner mantenga la conexion).
    hb_timer_ = create_wall_timer(1s, [this]() { link_->send_heartbeat(); });
    // Cada 5 s: texto de prueba, para ver que llega sin tener que publicar nada a mano.
    if (get_parameter("auto_test").as_bool()) {
      test_timer_ = create_wall_timer(5s, [this]() {
        link_->send_statustext(MAV_SEVERITY_INFO, "hola desde companion #" + std::to_string(count_++));
      });
    }
  }

 private:
  std::unique_ptr<GcsLink> link_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr from_gcs_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr to_gcs_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_, hb_timer_, test_timer_;
  int count_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GcsLinkNode>());
  rclcpp::shutdown();
  return 0;
}