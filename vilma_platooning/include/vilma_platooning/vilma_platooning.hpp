#ifndef VILMA_PLATOONING__VILMA_PLATOONING_HPP_
#define VILMA_PLATOONING__VILMA_PLATOONING_HPP_

#include "rclcpp/rclcpp.hpp"

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "etsi_its_msgs_utils/cam_access.hpp" // access functions

namespace vilma_platooning
{

  struct vehicle_states
  {
    int16_t speed;
    int16_t longitude;
    int16_t latitude;
    int16_t distance;
  } typedef vehicle_states_t;
  

  class VilmaPlatooning : public rclcpp::Node
  {

  public:

    VilmaPlatooning();

    void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);

  private:

    rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_sub_;

    vehicle_states_t target_vehicle_states_;
  };

} // namespace vilma_platooning
#endif // VILMA_PLATOONING__VILMA_PLATOONING_HPP_
