#ifndef vilma_obu_can__vilma_obu_can_HPP_
#define vilma_obu_can__vilma_obu_can_HPP_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>

#include <atomic>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/un.h>

#define CANVSTATE_CANID 0x123
#define CAN_OBD2 0x130
#define CAN_IMU 0x131

typedef struct can_frame can_frame_t;

typedef struct vehicle_imu_data{
    int16_t longitudinal_acceleration; // mm/s²
    u_int8_t longitudinal_acceleration_conf; // mm/s²
    int16_t lateral_acceleration; // mm/s²
    u_int8_t lateral_acceleration_conf; // mm/s²
    int16_t yaw_rate; // 0,01 degree per second. 
    u_int8_t yaw_rate_conf; // mm/s²
    int16_t vertical_acceleration; // 0,01 degree per second. 
    u_int8_t vertical_acceleration_conf; // mm/s²
} vehicle_imu_data_t;

namespace vilma_obu_can
{
class VilmaObuCan : public rclcpp::Node
{
public:
  VilmaObuCan();
  ~VilmaObuCan();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void angular_vel_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void free_acc_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void vilma_velocity_callback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
  void send_can_timer_callback();
  int16_t safeAssignToInt16(double value);

  std::atomic<bool> node_exit_{false};

private:
  std::string can_out_;
  int socket_out_;
  struct sockaddr_can addr_out_;
  struct ifreq ifr_out_;

  bool is_new_data_;

  vehicle_imu_data_t vehicle_its_data_;

  uint16_t vehicle_speed_cm_per_s_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_angular_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_free_acc_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vilma_velocity_sub_;

  rclcpp::TimerBase::SharedPtr send_can_timer_;
};
}  // namespace vilma_obu_can
#endif  // vilma_obu_can__vilma_obu_can_HPP_
