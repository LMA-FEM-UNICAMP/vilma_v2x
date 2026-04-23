#include "rclcpp/rclcpp.hpp"
#include "vilma_obu_can/vilma_obu_can.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vilma_obu_can::VilmaObuCan>();

  while (rclcpp::ok() && !node->node_exit_)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sample rate = 50 Hz (20 ms) => Pooling at 100 Hz
  }
  rclcpp::shutdown();
  return 0;
}
