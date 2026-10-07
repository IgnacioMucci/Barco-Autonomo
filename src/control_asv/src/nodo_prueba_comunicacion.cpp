// Prueba de comunicacion companion <-> Pixhawk (ArduPilot AP_DDS)
// Pixhawk -> companion : /ap/time y /ap/battery/battery0 (suscripcion)
// companion -> Pixhawk : servicio /ap/prearm_check (request) -> respuesta del Pixhawk
#include <chrono>
#include <cstdint>
#include <memory>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class CommTest : public rclcpp::Node
{
public:
  CommTest() : Node("nodo_prueba_comunicacion")
  {
    // Los topicos de ArduPilot son best-effort
    auto qos = rclcpp::SensorDataQoS();

    time_sub_ = create_subscription<builtin_interfaces::msg::Time>(
      "/ap/time", qos,
      [this](builtin_interfaces::msg::Time::ConstSharedPtr) {
        last_rx_ = now();
        ++time_count_;
      });

    batt_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/ap/battery/battery0", qos,
      [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
        last_rx_ = now();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "[Pixhawk -> PC] bateria: %.2f V", msg->voltage);
      });

    prearm_cli_ = create_client<std_srvs::srv::Trigger>("/ap/prearm_check");

    timer_ = create_wall_timer(2s, [this]() { tick(); });
  }

private:
  void tick()
  {
    RCLCPP_INFO(get_logger(), "RX /ap/time: %.1f Hz | prearm_check: %u enviados, %u respondidos",
                (time_count_ - last_time_count_) / 2.0, srv_sent_, srv_ok_);
    last_time_count_ = time_count_;

    // Watchdog de recepcion
    if ((now() - last_rx_).seconds() > 3.0) {
      RCLCPP_WARN(get_logger(), "Sin datos del Pixhawk (revisar agente / red / DDS_ENABLE)");
    }

    // Envio PC -> Pixhawk y espera de respuesta
    if (!prearm_cli_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "/ap/prearm_check no disponible");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto t0 = now();
    ++srv_sent_;
    prearm_cli_->async_send_request(
      req,
      [this, t0](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
        auto res = fut.get();
        ++srv_ok_;
        RCLCPP_INFO(get_logger(),
                    "[PC -> Pixhawk -> PC] prearm_check: success=%s msg='%s' RTT=%.1f ms",
                    res->success ? "true" : "false", res->message.c_str(),
                    (now() - t0).seconds() * 1000.0);
      });
  }

  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr time_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batt_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr prearm_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_rx_{0, 0, RCL_ROS_TIME};
  uint32_t time_count_{0};
  uint32_t last_time_count_{0};
  uint32_t srv_sent_{0};
  uint32_t srv_ok_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommTest>());
  rclcpp::shutdown();
  return 0;
}