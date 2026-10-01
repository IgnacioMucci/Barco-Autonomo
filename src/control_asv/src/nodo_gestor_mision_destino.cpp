// nodo_gestor_mision_destino.cpp
// Manda un punto de destino GPS a la Pixhawk vía AP_DDS. En modo GUIDED,
// ArduPilot calcula toda la navegación (rumbo, velocidad, mezcla) para
// llegar a ese punto por su cuenta.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ardupilot_msgs/msg/global_position.hpp"  // mensaje que espera /ap/cmd_gps_pose
#include "ardupilot_msgs/srv/arm_motors.hpp"
#include "ardupilot_msgs/srv/mode_switch.hpp"

using namespace std::chrono_literals;

constexpr uint8_t ROVER_MODE_GUIDED = 15;

// coordinate_frame es un campo numérico que le dice a ArduPilot cómo
// interpretar latitude/longitude/altitude. 5 = MAV_FRAME_GLOBAL_INT,
// que es "lat/lon en grados WGS84, altitud sobre el nivel del mar".
constexpr uint8_t FRAME_GLOBAL_INT = 5;

class NodoGestorMisionDestino : public rclcpp::Node
{
public:
  NodoGestorMisionDestino() : Node("nodo_gestor_mision_destino")
  {
    // Publisher del waypoint. 
    gps_publisher_ = this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(
      "/ap/cmd_gps_pose", 10);

    arm_client_ = this->create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");
    mode_client_ = this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");

    armar_y_cambiar_modo(); // mismo helper que en el nodo de cmd_vel

    // Acá el timer es más que nada para simplificar. En un nodo real, probablemente publicamos el waypoint
    // una sola vez, o solo cuando cambia el objetivo.
    timer_ = this->create_wall_timer(
      1s, std::bind(&NodoGestorMisionDestino::enviar_waypoint, this));

    RCLCPP_INFO(this->get_logger(), "Enviando waypoint vía /ap/cmd_gps_pose...");
  }

private:
  void armar_y_cambiar_modo()
  {
    // Mismo mecanismo de servicios que en nodo_cmd_vel.cpp: esperamos a que
    // el servicio exista, armamos el Request, lo mandamos sin bloquear.
    if (mode_client_->wait_for_service(5s)) {
      auto mode_req = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
      mode_req->mode = ROVER_MODE_GUIDED;
      mode_client_->async_send_request(mode_req);
    } else {
      RCLCPP_WARN(this->get_logger(), "Servicio /ap/mode_switch no disponible");
    }

    if (arm_client_->wait_for_service(5s)) {
      auto arm_req = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
      arm_req->arm = true;
      arm_client_->async_send_request(arm_req);
    } else {
      RCLCPP_WARN(this->get_logger(), "Servicio /ap/arm_motors no disponible");
    }
  }

  void enviar_waypoint()
  {
    auto mensaje = ardupilot_msgs::msg::GlobalPosition();

    mensaje.header.stamp = this->now();
    // "map": a diferencia de cmd_vel (que usaba "base_link", el marco del
    // propio vehículo), acá el punto está definido en un marco fijo del
    // mundo, no relativo al vehículo.
    mensaje.header.frame_id = "map";

    // Le decimos a ArduPilot cómo interpretar los campos de abajo.
    mensaje.coordinate_frame = FRAME_GLOBAL_INT;

    // Coordenadas del punto de destino, en grados decimales (WGS84).
    mensaje.latitude = -34.921450;
    mensaje.longitude = -57.954530;

    // Altitud en metros. Para el barco no importa, pero el campo es obligatorio en el mensaje.
    mensaje.altitude = 0.0f;

    gps_publisher_->publish(mensaje);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr gps_publisher_;
  rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr arm_client_;
  rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr mode_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodoGestorMisionDestino>());
  rclcpp::shutdown();
  return 0;
}