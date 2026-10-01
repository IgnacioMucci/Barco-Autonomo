/**
 * @file nodo_control_propulsion.cpp
 * @brief Nodo de Nivel Intermedio (dificultad de procesamiento). Calcula comandos de velocidad basados  
 *        en el error de distancia GPS y delega la decisión de la potencia (pwm) a la Pixhawk.
 */

#include "rclcpp/rclcpp.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_asv/utils/geo_utils.hpp"
#include <cmath>
#include <algorithm>

class NodoControlPropulsion : public rclcpp::Node {
public:
    NodoControlPropulsion() : Node("nodo_control_propulsion") {
        // Suscriptor al objetivo actual (latitud/longitud)
        target_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoint>(
            "/asv/objetivo_actual", 10,
            std::bind(&NodoControlPropulsion::target_cb, this, std::placeholders::_1));

        // Suscriptor a la telemetría GPS que provee MAVROS
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", 10,
            std::bind(&NodoControlPropulsion::gps_cb, this, std::placeholders::_1));

        // Publicador de velocidades hacia MAVROS
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
            
        RCLCPP_INFO(this->get_logger(), "Controlador de propulsión iniciado. Esperando objetivo...");
    }

private:
    // Esta función se ejecuta sola cuando el Gestor de Misión publica un nuevo destino
    void target_cb(const geographic_msgs::msg::GeoPoint::SharedPtr msg) {
        target_lat_ = msg->latitude;
        target_lon_ = msg->longitude;
        has_target_ = true;
        RCLCPP_INFO(this->get_logger(), "Nuevo objetivo recibido en el controlador.");
    }

    // Esta función se ejecuta sola automáticamente varias veces por segundo (ej. a 5Hz o 10Hz)
    // cada vez que la antena GPS reporta una nueva ubicación.
    void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // Si no tenemos a dónde ir, no calculamos nada
        if (!has_target_) return;

        // Calculamos la distancia real y el ángulo de rumbo (teniendo en cuenta curvatura de la tierra)
        auto [distancia, bearing_error] = asv_utils::calcular_distancia_rumbo(
            msg->latitude, msg->longitude, 
            target_lat_, target_lon_
        );

        // Preparamos el comando de velocidad de ROS 2
        auto twist_msg = geometry_msgs::msg::Twist();

        if (distancia > 2.0) { // Si estamos a más de 2 metros, seguimos navegando
            // std::clamp limita los valores para que el barco no acelere al infinito
            twist_msg.linear.x = std::clamp(distancia * 0.2, 0.0, 1.5); // Avance máx 1.5 m/s
            twist_msg.angular.z = std::clamp(bearing_error * 0.5, -0.8, 0.8); // Giro proporcional
        } else {
            // Si llegamos a menos de 2 metros, frenamos motores
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "¡Objetivo alcanzado, esperando nuevas órdenes!");
        }

        // Publicamos la orden; la Pixhawk 6X traducirá esto a PWM para los motores T200
        vel_pub_->publish(twist_msg);
    }
    
    // Variables internas guardadas en memoria
    rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    
    double target_lat_ = 0.0;
    double target_lon_ = 0.0;
    bool has_target_ = false;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // rclcpp::spin mantiene el nodo vivo y escuchando mensajes infinitamente
    rclcpp::spin(std::make_shared<NodoControlPropulsion>());
    rclcpp::shutdown();
    return 0;
}