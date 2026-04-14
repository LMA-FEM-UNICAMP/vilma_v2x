#ifndef vilma_traffic_light_rx_HPP_
#define vilma_traffic_light_rx_HPP_

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "etsi_its_spatem_ts_msgs/msg/spatem.hpp"
#include "etsi_its_msgs_utils/spatem_ts_access.hpp"  // access functions

#include "etsi_its_mapem_ts_msgs/msg/mapem.hpp"
#include "etsi_its_msgs_utils/mapem_ts_access.hpp"  // access functions

#include "vilma_traffic_light/traffic_light_structures.hpp"

namespace vilma_traffic_light_rx
{

class VilmaTrafficLightRx : public rclcpp::Node
{
  using SPATEM = etsi_its_spatem_ts_msgs::msg::SPATEM;
  using MAPEM = etsi_its_mapem_ts_msgs::msg::MAPEM;

public:
  VilmaTrafficLightRx(const rclcpp::NodeOptions&);

  void spatem_callback(const SPATEM::SharedPtr msg);
  void mapem_callback(const MAPEM::SharedPtr msg);
  std::string interpretMovementPhaseStateAsText(const uint8_t movement_phase_state);

private:
  rclcpp::Subscription<SPATEM>::SharedPtr spatem_sub_;
  rclcpp::Subscription<MAPEM>::SharedPtr mapem_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_to_change_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;

  /* Traffic light info */
  traffic_light_t traffic_light_status_;
};
}  // namespace vilma_traffic_light_rx
#endif  // vilma_traffic_light_rx_HPP_
