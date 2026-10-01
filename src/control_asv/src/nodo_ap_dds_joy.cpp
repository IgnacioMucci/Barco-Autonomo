
// Nodo "equivalente" al de MAVROS, pero usando el camino nativo AP_DDS.
// Acá NO mandamos µs directo: pensamos en µs y los traducimos a un eje
// normalizado justo antes de publicar, usando pwm_to_axis().

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"            // tipo de mensaje real que espera /ap/joy
#include "control_asv/utils/pwm_to_axis.hpp"  // nuestra función de traducción

using namespace std::chrono_literals;

class NodoApDdsJoy : public rclcpp::Node
{
public:
  NodoApDdsJoy() : Node("nodo_ap_dds_joy")
  {
    // El tópico real que escucha AP_DDS del lado de la Pixhawk es /ap/joy,
    // con tipo sensor_msgs/msg/Joy (no existe un tópico de PWM crudo acá).
    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/ap/joy", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&NodoApDdsJoy::enviar_pwm, this));

    RCLCPP_INFO(this->get_logger(), "Enviando override vía /ap/joy (AP_DDS)...");
  }

private:
  void enviar_pwm()
  {
    auto mensaje = sensor_msgs::msg::Joy();
    mensaje.header.stamp = this->now(); // Joy espera timestamp en el header

    // 4 ejes inicializados en 0.0 (equivalente al "canales(8, 0)" de antes,
    // pero acá Joy típicamente usa 4 ejes, no 8).
    std::vector<float> ejes(4, 0.0f);

    // Igual que antes: pensamos el valor en µs y lo convertimos al vuelo.
    ejes[0] = asv_utils::pwm_to_axis(1600);  // índice 0 -> canal 1 (motor izq.)
    ejes[2] = asv_utils::pwm_to_axis(1600);  // índice 2 -> canal 3 (motor der.)

    mensaje.axes = ejes;
    joy_publisher_->publish(mensaje);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodoApDdsJoy>());
  rclcpp::shutdown();
  return 0;
}