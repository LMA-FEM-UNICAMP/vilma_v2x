#include "vilma_platooning/vilma_platooning.hpp"

#include "haversine/haversine.h"

#include <chrono>

using namespace std::chrono_literals;

namespace vilma_platooning
{
    VilmaPlatooning::VilmaPlatooning() : Node("vilma_platooning")
    {
        /// Placeholders
        using std::placeholders::_1;

        /// Parameters
        this->declare_parameter("platooning_period_ms", 10);
        this->declare_parameter("hmi_update_period_ms", 500);

        /// Initialization
        platooning_state_ = false;

        /// ROS2 entities
        rclcpp::SubscriptionOptions sub_options;

        platooning_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        sub_options.callback_group = platooning_cb_group_;

        platooning_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(this->get_parameter("platooning_period_ms").as_int()),
            std::bind(&VilmaPlatooning::platooning_callback, this),
            platooning_cb_group_);

        hmi_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(this->get_parameter("hmi_update_period_ms").as_int()),
            std::bind(&VilmaPlatooning::hmi_update, this),
            platooning_cb_group_);

        control_mode_command_cli_ = this->create_client<ControlModeCommand>("/control/control_mode_request",
                                                                            rmw_qos_profile_services_default, platooning_cb_group_);

        control_mode_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", 10, std::bind(&VilmaPlatooning::control_mode_callback, this, _1),
            sub_options);

        velocity_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10, std::bind(&VilmaPlatooning::velocity_report_callback, this, _1),
            sub_options);

        cam_sub_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
            "/cam/in", 10, std::bind(&VilmaPlatooning::cam_callback, this, _1),
            sub_options);

        platooning_engage_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "/platooning/engage", 10, std::bind(&VilmaPlatooning::platooning_engage_callback, this, _1),
            sub_options);

        hmi_target_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/target_speed", 1);
        hmi_follower_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/follower_speed", 1);
        hmi_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/distance", 1);
        hmi_status_pub_ = this->create_publisher<std_msgs::msg::String>("/hmi/status", 1);
    }

    void VilmaPlatooning::control_mode_callback(const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg)
    {
        vehicle_control_mode_mutex_.lock();
        vehicle_control_mode_ = msg->mode;
        vehicle_control_mode_mutex_.unlock();
    }

    void VilmaPlatooning::velocity_report_callback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
    {
        following_vehicle_states_mutex_.lock();
        following_vehicle_states_.speed = msg->longitudinal_velocity;
        following_vehicle_states_mutex_.unlock();
    }

    void VilmaPlatooning::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
    {

        target_vehicle_states_mutex_.lock();

        target_vehicle_states_.speed = etsi_its_cam_msgs::access::getSpeed(*msg);
        target_vehicle_states_.longitude = etsi_its_cam_msgs::access::getLongitude(*msg);
        target_vehicle_states_.latitude = etsi_its_cam_msgs::access::getLatitude(*msg);

        following_vehicle_states_mutex_.lock();

        target_vehicle_states_.distance = calculate_distance(target_vehicle_states_.longitude,
                                                             target_vehicle_states_.longitude,
                                                             following_vehicle_states_.longitude,
                                                             following_vehicle_states_.longitude);

        following_vehicle_states_mutex_.unlock();
        target_vehicle_states_mutex_.unlock();
    }

    void VilmaPlatooning::platooning_engage_callback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        bool change_control_mode_result;

        switch (msg->data)
        {
        case VilmaPlatooning::PLATOONING_ENABLE:

            platooning_state_mutex_.lock();
            platooning_state_ = VilmaPlatooning::PLATOONING_ENABLE;
            platooning_state_mutex_.unlock();

            change_control_mode_result = change_control_mode(ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY);
            break;

        case VilmaPlatooning::PLATOONING_PAUSE:

            platooning_state_mutex_.lock();
            platooning_state_ = VilmaPlatooning::PLATOONING_PAUSE;
            platooning_state_mutex_.unlock();

            change_control_mode_result = change_control_mode(ControlModeCommand::Request::MANUAL);
            break;

        case VilmaPlatooning::PLATOONING_DISABLE:

            platooning_state_mutex_.lock();
            platooning_state_ = VilmaPlatooning::PLATOONING_DISABLE;
            platooning_state_mutex_.unlock();

            change_control_mode_result = change_control_mode(ControlModeCommand::Request::MANUAL);
            break;

        default:

            platooning_state_mutex_.lock();
            platooning_state_ = VilmaPlatooning::PLATOONING_PAUSE;
            platooning_state_mutex_.unlock();

            RCLCPP_WARN(this->get_logger(), "Unknow platooning engage command");
            change_control_mode_result = change_control_mode(ControlModeCommand::Request::MANUAL);
            break;
        }

        if (change_control_mode_result)
        {
            RCLCPP_INFO(this->get_logger(), "Control mode changed sucessfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Can't change control mode");
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

        int8_t platooning_state;

        platooning_state_mutex_.unlock();
        platooning_state = platooning_state_;
        platooning_state_mutex_.unlock();

        std_msgs::msg::Float32 follower_speed_hmi;

        following_vehicle_states_mutex_.lock();
        follower_speed_hmi.data = following_vehicle_states_.speed;
        following_vehicle_states_mutex_.unlock();

        hmi_follower_speed_pub_->publish(follower_speed_hmi);

        std_msgs::msg::Float32 target_speed_hmi;
        std_msgs::msg::Float32 distance_hmi;

        target_vehicle_states_mutex_.lock();
        target_speed_hmi.data = target_vehicle_states_.speed;
        distance_hmi.data = target_vehicle_states_.distance;
        target_vehicle_states_mutex_.unlock();

        hmi_follower_speed_pub_->publish(target_speed_hmi);
        hmi_target_speed_pub_->publish(distance_hmi);

        std_msgs::msg::String status_hmi;

        std::string platooning_status;

        switch (platooning_state)
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

        vehicle_control_mode_mutex_.lock();

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

        vehicle_control_mode_mutex_.unlock();

        status_hmi.data = std::string(platooning_status + " | " + control_mode_status);

        hmi_status_pub_->publish(status_hmi);
    }

    void VilmaPlatooning::platooning_callback()
    {

        int8_t platooning_state;

        platooning_state_mutex_.unlock();
        platooning_state = platooning_state_;
        platooning_state_mutex_.unlock();

        if (platooning_state)
        {
            vehicle_states_t follower_states;
            vehicle_states_t target_states;

            following_vehicle_states_mutex_.lock();
            follower_states = following_vehicle_states_;
            following_vehicle_states_mutex_.unlock();

            target_vehicle_states_mutex_.lock();
            target_states = target_vehicle_states_;
            target_vehicle_states_mutex_.unlock();

            // ! Run platooning control
        }
    }

} // namespace VilmaPlatooning
