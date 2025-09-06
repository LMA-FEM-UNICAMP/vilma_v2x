#include "vilma_platooning/vilma_platooning.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace vilma_platooning
{
    VilmaPlatooning::VilmaPlatooning() : Node("vilma_platooning")
    {

        platooning_state_ = false;

        using std::placeholders::_1;

        platooning_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VilmaPlatooning::platooning_callback, this));

        control_mode_command_cli_ = this->create_client<ControlModeCommand>("/control/control_mode_request");

        control_mode_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", 10, std::bind(&VilmaPlatooning::control_mode_callback, this, _1));

        velocity_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10, std::bind(&VilmaPlatooning::velocity_report_callback, this, _1));

        cam_sub_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
            "/cam/in", 10, std::bind(&VilmaPlatooning::cam_callback, this, _1));

        platooning_engage_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "/platooning/engage", 10, std::bind(&VilmaPlatooning::platooning_engage_callback, this, _1));

        hmi_target_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/target_speed", 1);
        hmi_follower_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/follower_speed", 1);
        hmi_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/distance", 1);
        hmi_status_pub_ = this->create_publisher<std_msgs::msg::String>("/hmi/status", 1);
    }

    void VilmaPlatooning::control_mode_callback(const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg)
    {
        vehicle_control_mode_ = msg->mode;
    }

    void VilmaPlatooning::velocity_report_callback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
    {
        following_vehicle_states_.speed = msg->longitudinal_velocity;
    }

    void VilmaPlatooning::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
    {
        target_vehicle_states_.speed = etsi_its_cam_msgs::access::getSpeed(*msg);
        target_vehicle_states_.longitude = etsi_its_cam_msgs::access::getLongitude(*msg);
        target_vehicle_states_.latitude = etsi_its_cam_msgs::access::getLatitude(*msg);
    }

    void VilmaPlatooning::platooning_engage_callback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        bool _;

        switch (msg->data)
        {
        case VilmaPlatooning::PLATOONING_ENABLE:

            platooning_state_ = VilmaPlatooning::PLATOONING_ENABLE;
            _ = change_control_mode(ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY);
            break;

        case VilmaPlatooning::PLATOONING_PAUSE:

            platooning_state_ = VilmaPlatooning::PLATOONING_PAUSE;
            _ = change_control_mode(ControlModeCommand::Request::MANUAL);
            break;

        case VilmaPlatooning::PLATOONING_DISABLE:

            platooning_state_ = VilmaPlatooning::PLATOONING_DISABLE;
            _ = change_control_mode(ControlModeCommand::Request::MANUAL);
            break;

        default:
            platooning_state_ = VilmaPlatooning::PLATOONING_PAUSE;
            RCLCPP_WARN(this->get_logger(), "Unknow platooning engage command");
            _ = change_control_mode(ControlModeCommand::Request::MANUAL);
            break;
        }
    }

    bool VilmaPlatooning::change_control_mode(const uint8_t control_mode)
    {
        auto change_control_mode_request = std::make_shared<ControlModeCommand::Request>();

        while (!control_mode_command_cli_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            }
            RCLCPP_INFO(this->get_logger(), "Service not available yet, waiting again...");
        }

        change_control_mode_request->mode = control_mode;

        auto change_control_mode_result = control_mode_command_cli_->async_send_request(change_control_mode_request);

        if (rclcpp::spin_until_future_complete(shared_from_this(), change_control_mode_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (change_control_mode_result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Control mode change successful.");
                return 1;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Control mode change unsuccessful.");
                return 0;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service to change control mode.");
            return 0;
        }
    }

    void VilmaPlatooning::hmi_update()
    {

        std_msgs::msg::Float32 follower_speed_hmi;
        follower_speed_hmi.data = following_vehicle_states_.speed;
        hmi_follower_speed_pub_->publish(follower_speed_hmi);

        std_msgs::msg::Float32 target_speed_hmi;
        target_speed_hmi.data = target_vehicle_states_.speed;
        hmi_follower_speed_pub_->publish(target_speed_hmi);

        std_msgs::msg::Float32 distance_hmi;
        distance_hmi.data = target_vehicle_states_.distance;
        hmi_target_speed_pub_->publish(distance_hmi);

        std_msgs::msg::String status_hmi;

        std::string platooning_status;

        switch (platooning_state_)
        {
        case VilmaPlatooning::PLATOONING_DISABLE:
            platooning_status = "Platooning disabled";
            break;
        case VilmaPlatooning::PLATOONING_PAUSE:
            platooning_status = "Platooning paused";
            break;
        case VilmaPlatooning::PLATOONING_ENABLE:
            platooning_status = "Platooning enabled";
            break;

        default:
            platooning_status = "Platooning error";
            break;
        }

        std::string control_mode_status;

        switch (vehicle_control_mode_)
        {
        case autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY:
            control_mode_status = "Velocity control active";
            break;
        case autoware_vehicle_msgs::msg::ControlModeReport::MANUAL:
            control_mode_status = "Manual control";
            break;
        default:
            control_mode_status = "Control error";
            break;
        }

        status_hmi.data = std::string(platooning_status + " | " + control_mode_status);

        hmi_status_pub_->publish(status_hmi);
    }

    void VilmaPlatooning::platooning_callback()
    {

        if (platooning_state_)
        {
            // ! Run platooning control
        }
    }

} // namespace VilmaPlatooning
