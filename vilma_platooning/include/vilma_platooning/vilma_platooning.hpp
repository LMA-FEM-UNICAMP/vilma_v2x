#ifndef VILMA_PLATOONING__VILMA_PLATOONING_HPP_
#define VILMA_PLATOONING__VILMA_PLATOONING_HPP_

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "etsi_its_msgs_utils/cam_access.hpp" // access functions

#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;

namespace vilma_platooning
{

  struct vehicle_states
  {
    float speed;
    float longitude;
    float latitude;
    float distance;
  } typedef vehicle_states_t;

  class VilmaPlatooning : public rclcpp::Node
  {

  public:
    constexpr static uint8_t PLATOONING_DISABLE = 1; // Do nothing
    constexpr static uint8_t PLATOONING_ENABLE = 2;  // Start platooning
    constexpr static uint8_t PLATOONING_PAUSE = 3;   // Run platooning but disengage vehicle

    VilmaPlatooning();

    void hmi_update();

    bool change_control_mode(const uint8_t control_mode);

    void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);

    void platooning_engage_callback(const std_msgs::msg::UInt16::SharedPtr msg);

    void control_mode_callback(const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg);

    void velocity_report_callback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

    void platooning_callback();

  private:

    /// Platooning control
    int8_t platooning_state_;
    int8_t vehicle_control_mode_;

    vehicle_states_t target_vehicle_states_;
    vehicle_states_t following_vehicle_states_;

    /// Shared variables mutexes
    std::mutex target_vehicle_states_mutex_;
    std::mutex following_vehicle_states_mutex_;
    std::mutex platooning_state_mutex_;
    std::mutex vehicle_control_mode_mutex_;

    /// ROS objects
    rclcpp::CallbackGroup::SharedPtr platooning_cb_group_;

    rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_sub_;

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr platooning_engage_sub_;

    rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hmi_target_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hmi_follower_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hmi_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hmi_status_pub_;

    rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_commmand_pub_;

    rclcpp::Client<ControlModeCommand>::SharedPtr control_mode_command_cli_;

    rclcpp::TimerBase::SharedPtr platooning_timer_;
    rclcpp::TimerBase::SharedPtr hmi_timer_;
  };

} // namespace vilma_platooning
#endif // VILMA_PLATOONING__VILMA_PLATOONING_HPP_
