// Nodo que manda PWM crudo (en microsegundos) a la Pixhawk usando MAVROS.
// MAVROS traduce esto internamente al mensaje MAVLink RC_CHANNELS_OVERRIDE,
// que es el mismo mecanismo que dispara el modo PassThru en el firmware.

#include <chrono>   // para poder escribir "100ms"
#include <memory>   // para std::make_shared
#include <vector>   // no lo usamos directo acá, pero queda por si se agregan canales

#include "rclcpp/rclcpp.hpp"                       // API base de ROS 2 para C++
#include "mavros_msgs/msg/override_rc_in.hpp"      // el mensaje real que espera MAVROS

using namespace std::chrono_literals; // habilita el sufijo "ms" en 100ms

// El "operario": un Nodo de ROS 2 que publica el override cada 100ms.
class NodoMavros : public rclcpp::Node
{
public:
  NodoMavros() : Node("nodo_mavros") // nombre del nodo en el grafo de ROS 2
  {
    // Creamos el publisher. El tópico "/mavros/rc/override" es el que
    // levanta el nodo mavros (no la Pixhawk directamente) para reenviar
    // esto como MAVLink RC_CHANNELS_OVERRIDE.
    pwm_publisher_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
      "/mavros/rc/override", 10);

    // Timer que llama a enviar_pwm() 10 veces por segundo.
    timer_ = this->create_wall_timer(
      100ms, std::bind(&NodoMavros::enviar_pwm, this));

    RCLCPP_INFO(this->get_logger(), "Enviando override crudo vía MAVROS...");
  }

private:
  void enviar_pwm()
  {
    // OverrideRCIn trae internamente un array fijo "channels" (normalmente
    // de 8 posiciones). Cada elemento es un uint16_t en microsegundos.
    auto mensaje = mavros_msgs::msg::OverrideRCIn();

    // channels[0] = canal 1 -> lo mapeaste a SERVO1_FUNCTION=51 (motor izq.)
    mensaje.channels[0] = 1600;

    // channels[2] = canal 3 -> lo mapeaste a SERVO3_FUNCTION=53 (motor der.)
    mensaje.channels[2] = 1600;

    // Los canales que no tocamos quedan en 0 por default en la mayoría de
    // versiones de mavros_msgs; 0 significa "no hacer override de este canal".
    // (Si tu versión de MAVROS usa 65535/UINT16_MAX como "sin override",
    // hay que setearlo explícito para los canales que no querés tocar).

    pwm_publisher_->publish(mensaje);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr pwm_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                              // arranca ROS 2
  rclcpp::spin(std::make_shared<NodoMavros>());   // crea el nodo y lo deja publicando
  rclcpp::shutdown();                                     // al hacer Ctrl+C, cierra todo prolijo
  return 0;
}