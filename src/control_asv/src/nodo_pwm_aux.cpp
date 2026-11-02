/**
 * @file nodo_pwm_aux.cpp
 * @brief Nodo de ROS 2 que fija el PWM de las salidas AUX (FMU PWM OUT) de la Pixhawk
 *        enviando MAV_CMD_DO_SET_SERVO a través de MAVROS.
 */

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "mavros_msgs/srv/command_long.hpp"

using CommandLong = mavros_msgs::srv::CommandLong;

// Número de comando de MAVLink (MAV_CMD_DO_SET_SERVO = 183)
constexpr uint16_t MAV_CMD_DO_SET_SERVO = 183;

class NodoPwmAux : public rclcpp::Node
{
public:
  NodoPwmAux() : Node("nodo_pwm_aux")
  {
    // Parámetros configurables desde la línea de comandos o un launch
    canal_inicial_ = this->declare_parameter<int>("canal_inicial", 9);   // 9 = AUX1
    pwm_min_       = this->declare_parameter<int>("pwm_min", 1000);
    pwm_max_       = this->declare_parameter<int>("pwm_max", 2000);
    const std::string servicio =
      this->declare_parameter<std::string>("servicio", "/mavros/cmd/command");

    // Cliente del servicio de MAVROS que envía COMMAND_LONG a la placa
    cliente_ = this->create_client<CommandLong>(servicio);

    // Suscriptor: data[0] -> canal_inicial, data[1] -> canal_inicial + 1, ...
    pwm_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/pwm_aux", 10,
      std::bind(&NodoPwmAux::pwm_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Nodo PWM AUX iniciado. Canal inicial: %d, rango: %d-%d us, servicio: %s",
      canal_inicial_, pwm_min_, pwm_max_, servicio.c_str());
  }

private:
  /**
   * @brief Se ejecuta al recibir una lista de valores PWM.
   * @details Envía un DO_SET_SERVO por cada canal cuyo valor cambió.
   */
  void pwm_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (!cliente_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "El servicio de MAVROS no está disponible (¿MAVROS conectado a la placa?)");
      return;
    }

    if (ultimo_pwm_.size() < msg->data.size()) {
      ultimo_pwm_.resize(msg->data.size(), -1);  // -1 = nunca enviado
    }

    for (size_t i = 0; i < msg->data.size(); ++i) {
      const int pwm = std::clamp<int>(msg->data[i], pwm_min_, pwm_max_);
      if (pwm == ultimo_pwm_[i]) {
        continue;  // sin cambios, no reenviamos
      }
      enviar_servo(i, canal_inicial_ + static_cast<int>(i), pwm);
    }
  }

  /**
   * @brief Arma y envía un COMMAND_LONG con MAV_CMD_DO_SET_SERVO.
   * @param indice Posición dentro del arreglo recibido.
   * @param canal  Número de canal de servo en ArduPilot (9 = AUX1).
   * @param pwm    Ancho de pulso en microsegundos.
   */
  void enviar_servo(size_t indice, int canal, int pwm)
  {
    auto req = std::make_shared<CommandLong::Request>();
    req->broadcast    = false;
    req->command      = MAV_CMD_DO_SET_SERVO;
    req->confirmation = 0;
    req->param1       = static_cast<float>(canal);  // canal de servo
    req->param2       = static_cast<float>(pwm);    // PWM en us
    req->param3       = 0.0f;
    req->param4       = 0.0f;
    req->param5       = 0.0f;
    req->param6       = 0.0f;
    req->param7       = 0.0f;

    // Envío asíncrono: la respuesta llega por callback y no bloquea el nodo
    cliente_->async_send_request(req,
      [this, indice, canal, pwm](rclcpp::Client<CommandLong>::SharedFuture futuro) {
        const auto resp = futuro.get();
        if (resp->success) {
          ultimo_pwm_[indice] = pwm;
          RCLCPP_INFO(this->get_logger(), "Canal %d -> %d us (aceptado)", canal, pwm);
        } else {
          RCLCPP_WARN(this->get_logger(),
            "Canal %d rechazado (resultado=%u). Revisar que SERVO%d_FUNCTION = 0.",
            canal, resp->result, canal);
        }
      });
  }

  int canal_inicial_;
  int pwm_min_;
  int pwm_max_;
  std::vector<int> ultimo_pwm_;
  rclcpp::Client<CommandLong>::SharedPtr cliente_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr pwm_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodoPwmAux>());
  rclcpp::shutdown();
  return 0;
}