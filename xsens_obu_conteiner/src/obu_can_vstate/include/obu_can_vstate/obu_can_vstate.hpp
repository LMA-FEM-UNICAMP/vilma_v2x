#ifndef obu_can_vstate__obu_can_vstate_HPP_
#define obu_can_vstate__obu_can_vstate_HPP_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/un.h>

typedef struct can_frame can_frame_t;


typedef struct vehicle_imu_data{
    int16_t longitudinal_acceleration; // mm/s²
    u_int8_t longitudinal_acceleration_conf; // mm/s²
    int16_t lateral_acceleration; // mm/s²
    u_int8_t lateral_acceleration_conf; // mm/s²
    int16_t yaw_rate; // 0,01 degree per second. 
    u_int8_t yaw_rate_conf; // mm/s²
} vehicle_imu_data_t;

namespace obu_can_vstate
{
class ObuCanVState : public rclcpp::Node
{
public:
  ObuCanVState();
  ~ObuCanVState();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void angular_vel_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void free_acc_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void send_can_timer_callback();

private:
  std::string can_out_;
  int socket_out_;
  struct sockaddr_can addr_out_;
  struct ifreq ifr_out_;

  bool is_new_data_;

  u_int32_t obu_vstate_can_id_;

  vehicle_imu_data_t vehicle_its_data_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_angular_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_free_acc_sub_;

  rclcpp::TimerBase::SharedPtr send_can_timer_;
};
}  // namespace obu_can_vstate
#endif  // obu_can_vstate__obu_can_vstate_HPP_
