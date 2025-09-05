#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vilma_platooning/vilma_platooning.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto vilma_platooning_node = std::make_shared<vilma_platooning::VilmaPlatooning>();

    //* Creating multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor mt_executor;

    //* Adding node to executor
    mt_executor.add_node(vilma_platooning_node);

    RCLCPP_INFO(vilma_platooning_node->get_logger(), "Starting VILMA's platooning node...");
    mt_executor.spin();
    RCLCPP_INFO(vilma_platooning_node->get_logger(), "Shutting down VILMA's platooning node. \n");

    rclcpp::shutdown();
    return 0;
}