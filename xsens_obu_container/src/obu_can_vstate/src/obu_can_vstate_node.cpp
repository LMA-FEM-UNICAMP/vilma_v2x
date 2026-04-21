#include "rclcpp/rclcpp.hpp"
#include "obu_can_vstate/obu_can_vstate.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<obu_can_vstate::ObuCanVState>();

  while (rclcpp::ok() && !node->node_exit_)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  rclcpp::shutdown();
  return 0;
}
