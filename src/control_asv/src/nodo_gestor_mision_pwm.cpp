/**
 * @file nodo_traductor_mission_planner.cpp
 * @brief Nodo de ROS 2 que traduce los comandos recibidos desde Mission Planner (vía MAVROS/MAVLink)
 *        a instrucciones que la placa de navegación (ArduRover) puede procesar.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

class NodoTraductorMissionPlanner : public rclcpp::Node
{
public:
  NodoTraductorMissionPlanner() : Node("nodo_traductor_mission_planner")
  {
    // Suscriptor que escucha los comandos de velocidad genéricos (que podrían venir mapeados desde Mission Planner)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_planner", 10,
      std::bind(&NodoTraductorMissionPlanner::cmd_vel_callback, this, std::placeholders::_1));

    // Publicador hacia MAVROS para convertir las órdenes a PWM crudo / Override de RC si se requiere control directo
    rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
      "/mavros/rc/override", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo Traductor de Mission Planner iniciado correctamente.");
  }

private:
  /**
   * @brief Callback que se ejecuta al recibir un mensaje de velocidad (Twist)
   * @details Traduce velocidades lineales y angulares a valores de canales PWM aptos para la placa.
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double lineal_x = msg->linear.x;   // Velocidad de avance (m/s)
    double angular_z = msg->angular.z; // Velocidad de giro (rad/s)

    // Lógica básica de traducción de velocidad a PWM (Ejemplo diferencial / Skid Steering)
    // Asumimos un centro de 1500 µs, sumando/restando según lineal y angular
    int base_pwm = 1500;
    int pwm_izq = base_pwm + static_cast<int>(lineal_x * 200.0 - angular_z * 100.0);
    int pwm_der = base_pwm + static_cast<int>(lineal_x * 200.0 + angular_z * 100.0);

    // Limitamos los valores de PWM al rango seguro de los propulsores T200 (1100 a 1900 µs)
    pwm_izq = std::clamp(pwm_izq, 1100, 1900);
    pwm_der = std::clamp(pwm_der, 1100, 1900);

    // Armamos el mensaje de MAVROS para enviar el override a la placa
    auto override_msg = mavros_msgs::msg::OverrideRCIn();
    
    // Canal 1 (Motor izquierdo)
    override_msg.channels[0] = static_cast<uint16_t>(pwm_izq);
    // Canal 3 (Motor derecho)
    override_msg.channels[2] = static_cast<uint16_t>(pwm_der);

    // Publicamos la instrucción traducida hacia MAVROS para que la envíe a la placa de navegación
    rc_override_pub_->publish(override_msg);

    RCLCPP_INFO(this->get_logger(), "Traducido -> Lineal: %.2f | Angular: %.2f ==> PWM Izq: %d, PWM Der: %d",
      lineal_x, angular_z, pwm_izq, pwm_der);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodoTraductorMissionPlanner>());
  rclcpp::shutdown();
  return 0;
}