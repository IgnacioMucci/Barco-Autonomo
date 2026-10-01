#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

class NodoGestorMisionDistanciaRumbo : public rclcpp::Node {
public:
    NodoGestorMisionDistanciaRumbo() : Node("nodo_gestor_mision_dist_rumbo") {
        // Suscriptor a la lista de waypoints que envía Mission Planner vía MAVROS
        wp_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
            "/mavros/mission/waypoints", 10,
            std::bind(&NodoGestorMisionDistanciaRumbo::waypoint_cb, this, std::placeholders::_1));

        // Publicador del objetivo actual para el nodo de propulsión
        target_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoint>(
            "/asv/objetivo_actual", 10);

        RCLCPP_INFO(this->get_logger(), "Gestor de misión traducido a distancia y ángulo (rumbo) iniciado. Esperando ruta de Mission Planner...");
    }

private:
    void waypoint_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
        // Buscamos el waypoint actual marcado como activo
        for (const auto& wp : msg->waypoints) {
            if (wp.is_current) {
                auto target = geographic_msgs::msg::GeoPoint();
                target.latitude = wp.x_lat;
                target.longitude = wp.y_long;
                target.altitude = wp.z_alt;

                target_pub_->publish(target);
                RCLCPP_INFO(this->get_logger(), "Nuevo objetivo: Lat %.6f, Lon %.6f", target.latitude, target.longitude);
                break;
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr wp_sub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoint>::SharedPtr target_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodoGestorMisionDistanciaRumbo>());
    rclcpp::shutdown();
    return 0;
}