#include "rclcpp/rclcpp.hpp"
#include "obu_can_vstate/obu_can_vstate.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<obu_can_vstate::ObuCanVState>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
