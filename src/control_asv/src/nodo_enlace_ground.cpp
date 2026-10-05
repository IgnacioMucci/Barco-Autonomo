// nodo_enlace_ground.cpp - Enlace companion <-> ground usando MAVROS como puente a la radio.
//
// MAVROS corre con gcs_url apuntando a la radio (ver enlace_ground_launch.py).
// Este nodo NO abre el puerto serie: solo habla ROS 2 con MAVROS.
//
// COMPANION -> GROUND
//   /mavros/statustext/send   mavros_msgs/StatusText -> STATUSTEXT         (Mission Planner: pestana Mensajes)
//   /mavros/debug_value/send  mavros_msgs/DebugValue -> NAMED_VALUE_FLOAT  (Mission Planner: Ctrl+F > MAVLink Inspector)
//
// GROUND -> COMPANION
//   /uas1/mavlink_source (mavros_msgs/Mavlink) trae todo lo que pasa por el router de MAVROS.
//   Se decodifica y se republica en:
//     /asv/ground/texto   std_msgs/String
//     /asv/ground/valor   mavros_msgs/DebugValue

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/debug_value.hpp"
#include "mavros_msgs/msg/mavlink.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "std_msgs/msg/string.hpp"

using mavros_msgs::msg::DebugValue;
using mavros_msgs::msg::Mavlink;
using mavros_msgs::msg::StatusText;

// ============================ DECODIFICADOR MAVLINK ============================
// BEGIN_DECODER
namespace mav
{
constexpr uint32_t MSG_HEARTBEAT = 0;
constexpr uint32_t MSG_NAMED_VALUE_FLOAT = 251;
constexpr uint32_t MSG_NAMED_VALUE_INT = 252;
constexpr uint32_t MSG_STATUSTEXT = 253;

enum class Tipo { HEARTBEAT, STATUSTEXT, NAMED_VALUE_FLOAT, NAMED_VALUE_INT };

struct Decodificado
{
  Tipo tipo{};
  uint8_t mav_type{0};
  uint8_t autopilot{0};
  uint8_t severidad{0};
  std::string texto;
  std::string nombre;
  float valor_f{0.0f};
  int32_t valor_i{0};
};

inline std::string cstr(const uint8_t * b, size_t max)
{
  size_t n = 0;
  while (n < max && b[n] != 0) {++n;}
  return std::string(reinterpret_cast<const char *>(b), n);
}

// Payload MAVLink2: sin ceros finales -> se rellena a 255 bytes. Little-endian.
inline std::optional<Decodificado> decodificar(uint32_t msgid, const uint8_t * payload, size_t len)
{
  std::array<uint8_t, 255> p{};
  std::memcpy(p.data(), payload, std::min(len, p.size()));

  Decodificado d;
  switch (msgid) {
    case MSG_HEARTBEAT:
      d.tipo = Tipo::HEARTBEAT;
      d.mav_type = p[4];
      d.autopilot = p[5];
      return d;
    case MSG_STATUSTEXT:
      d.tipo = Tipo::STATUSTEXT;
      d.severidad = p[0];
      d.texto = cstr(&p[1], 50);
      return d;
    case MSG_NAMED_VALUE_FLOAT:
      d.tipo = Tipo::NAMED_VALUE_FLOAT;
      d.nombre = cstr(&p[8], 10);
      std::memcpy(&d.valor_f, &p[4], sizeof(float));
      return d;
    case MSG_NAMED_VALUE_INT:
      d.tipo = Tipo::NAMED_VALUE_INT;
      d.nombre = cstr(&p[8], 10);
      std::memcpy(&d.valor_i, &p[4], sizeof(int32_t));
      return d;
    default:
      return std::nullopt;
  }
}
}  // namespace mav
// END_DECODER
// ===============================================================================

class NodoEnlaceGround : public rclcpp::Node
{
public:
  NodoEnlaceGround()
  : Node("nodo_enlace_ground")
  {
    declare_parameter<double>("periodo", 1.0);        // s entre envios
    declare_parameter<std::string>("uas", "uas1");    // prefijo del bus MAVLink de MAVROS
    declare_parameter<int>("tgt_system", 1);          // sysid del vehiculo

    const double periodo = get_parameter("periodo").as_double();
    const std::string uas = get_parameter("uas").as_string();
    tgt_system_ = get_parameter("tgt_system").as_int();

    // ---- TX: companion -> ground ----
    pub_texto_ = create_publisher<StatusText>("/mavros/statustext/send", rclcpp::SensorDataQoS());
    pub_valor_ = create_publisher<DebugValue>("/mavros/debug_value/send", rclcpp::QoS(10));

    // ---- RX: ground -> companion ----
    // El router de MAVROS publica /uasN/mavlink_source con best_effort / volatile.
    auto qos_raw = rclcpp::QoS(100).best_effort().durability_volatile();
    sub_raw_ = create_subscription<Mavlink>(
      "/" + uas + "/mavlink_source", qos_raw,
      std::bind(&NodoEnlaceGround::mavlink_cb, this, std::placeholders::_1));
    pub_g_texto_ = create_publisher<std_msgs::msg::String>("/asv/ground/texto", 10);
    pub_g_valor_ = create_publisher<DebugValue>("/asv/ground/valor", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(periodo), std::bind(&NodoEnlaceGround::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Enlace listo. TX: /mavros/statustext/send + /mavros/debug_value/send | "
      "RX: /%s/mavlink_source (sysid != %d)", uas.c_str(), tgt_system_);
  }

  // ------------------------------------------------------------------ TX
  void enviar_texto(const std::string & texto, uint8_t severidad = StatusText::INFO)
  {
    StatusText m;
    m.header.stamp = now();
    m.severity = severidad;
    m.text = texto.substr(0, 50);   // MAVLink: max. 50 caracteres (usar ASCII)
    pub_texto_->publish(m);
  }

  void enviar_float(const std::string & nombre, float valor)
  {
    DebugValue m;
    m.header.stamp = now();
    m.type = DebugValue::TYPE_NAMED_VALUE_FLOAT;
    m.index = -1;
    m.array_id = -1;
    m.name = nombre.substr(0, 10);  // max. 10 caracteres
    m.value_float = valor;
    pub_valor_->publish(m);
  }

  void enviar_int(const std::string & nombre, int32_t valor)
  {
    DebugValue m;
    m.header.stamp = now();
    m.type = DebugValue::TYPE_NAMED_VALUE_INT;
    m.index = -1;
    m.array_id = -1;
    m.name = nombre.substr(0, 10);
    m.value_int = valor;
    pub_valor_->publish(m);
  }

private:
  void tick()
  {
    ++n_;
    if (pub_texto_->get_subscription_count() == 0 || pub_valor_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "MAVROS no esta escuchando statustext/send o debug_value/send "
        "(MAVROS caido, o falta mavros_extras / plugin debug_value en denylist)");
    }
    const std::string txt = "Hola desde companion #" + std::to_string(n_);
    enviar_texto(txt);
    enviar_float("ASV_CNT", static_cast<float>(n_));
    RCLCPP_INFO(get_logger(), "TX -> ground: \"%s\" | ASV_CNT=%d", txt.c_str(), n_);
  }

  // ------------------------------------------------------------------ RX
  void mavlink_cb(const Mavlink::ConstSharedPtr m)
  {
    if (m->framing_status != Mavlink::FRAMING_OK || m->sysid == tgt_system_) {
      return;   // paquete roto, o del vehiculo/MAVROS mismo
    }

    // payload64 (little-endian) -> bytes
    const size_t len = std::min<size_t>(m->len, m->payload64.size() * 8);
    std::array<uint8_t, 255> buf{};
    for (size_t i = 0; i < len && i < buf.size(); ++i) {
      buf[i] = static_cast<uint8_t>((m->payload64[i / 8] >> (8 * (i % 8))) & 0xFF);
    }

    const auto d = mav::decodificar(m->msgid, buf.data(), len);
    if (!d) {return;}
    const std::string origen = std::to_string(m->sysid) + "." + std::to_string(m->compid);

    switch (d->tipo) {
      case mav::Tipo::HEARTBEAT:
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "RX <- ground [%s] HEARTBEAT (mav_type=%u)", origen.c_str(), d->mav_type);
        break;

      case mav::Tipo::STATUSTEXT: {
          RCLCPP_INFO(get_logger(), "RX <- ground [%s] TEXTO: \"%s\"", origen.c_str(), d->texto.c_str());
          std_msgs::msg::String s;
          s.data = d->texto;
          pub_g_texto_->publish(s);
          break;
        }

      case mav::Tipo::NAMED_VALUE_FLOAT:
      case mav::Tipo::NAMED_VALUE_INT: {
          DebugValue v;
          v.header.stamp = now();
          v.index = -1;
          v.array_id = -1;
          v.name = d->nombre;
          if (d->tipo == mav::Tipo::NAMED_VALUE_FLOAT) {
            v.type = DebugValue::TYPE_NAMED_VALUE_FLOAT;
            v.value_float = d->valor_f;
            RCLCPP_INFO(get_logger(), "RX <- ground [%s] %s = %f", origen.c_str(), d->nombre.c_str(), d->valor_f);
          } else {
            v.type = DebugValue::TYPE_NAMED_VALUE_INT;
            v.value_int = d->valor_i;
            RCLCPP_INFO(get_logger(), "RX <- ground [%s] %s = %d", origen.c_str(), d->nombre.c_str(), d->valor_i);
          }
          pub_g_valor_->publish(v);
          break;
        }
    }
  }

  int tgt_system_{1};
  int n_{0};
  rclcpp::Publisher<StatusText>::SharedPtr pub_texto_;
  rclcpp::Publisher<DebugValue>::SharedPtr pub_valor_;
  rclcpp::Subscription<Mavlink>::SharedPtr sub_raw_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_g_texto_;
  rclcpp::Publisher<DebugValue>::SharedPtr pub_g_valor_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodoEnlaceGround>());
  rclcpp::shutdown();
  return 0;
}