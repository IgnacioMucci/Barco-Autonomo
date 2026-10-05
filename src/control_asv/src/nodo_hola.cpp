// nodo_hola.cpp
// QUEDÓ VIEJO, NO ANDA
// Manda un mensaje de texto a Mission Planner usando MAVROS.
// Publica mavros_msgs/StatusText en /mavros/statustext/send; MAVROS lo convierte
// en un STATUSTEXT de MAVLink. En Mission Planner aparece en la pestaña
// "Mensajes" del HUD.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/status_text.hpp"

using namespace std::chrono_literals;

class NodoHola : public rclcpp::Node
{
public:
  NodoHola() : Node("nodo_hola")
  {
    pub_ = this->create_publisher<mavros_msgs::msg::StatusText>(
      "/mavros/statustext/send", 10);

    timer_ = this->create_wall_timer(2s, std::bind(&NodoHola::enviar, this));
  }

private:
  void enviar()
  {
    mavros_msgs::msg::StatusText msg;
    msg.header.stamp = this->now();
    msg.severity = mavros_msgs::msg::StatusText::INFO;
    // MAVLink limita el texto a 50 caracteres.
    msg.text = "Hola desde companion #" + std::to_string(++n_);

    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Enviado: %s", msg.text.c_str());
  }

  int n_ = 0;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodoHola>());
  rclcpp::shutdown();
  return 0;
}