#include "vilma_platooning/vilma_platooning.hpp"

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

namespace vilma_platooning
{
    VilmaPlatooning::VilmaPlatooning() : Node("vilma_platooning")
    {
        /// Placeholders
        using std::placeholders::_1;

        /// Parameters
        this->declare_parameter("platooning_period_ms", 250);
        this->declare_parameter("hmi_update_period_ms", 500);
        this->declare_parameter("gnss_topic", "/gnss");

        /// Initialization
        platooning_state_.store(0);

        following_vehicle_states_.latitude = 0.0;
        following_vehicle_states_.longitude = 0.0;

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
            "/vehicle/status/control_mode", rclcpp::QoS{1}, std::bind(&VilmaPlatooning::control_mode_callback, this, _1),
            sub_options);

        velocity_report_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", rclcpp::QoS{1}, std::bind(&VilmaPlatooning::velocity_report_callback, this, _1),
            sub_options);

        cam_sub_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
            "/cam/out", rclcpp::QoS{1}, std::bind(&VilmaPlatooning::cam_callback, this, _1),
            sub_options);

        leader_gnss_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/leader/fix", rclcpp::QoS{1});

        follower_gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            this->get_parameter("gnss_topic").as_string(), rclcpp::QoS{1}, std::bind(&VilmaPlatooning::follower_gnss_callback, this, _1),
            sub_options);

        platooning_engage_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "/hmi/engage", rclcpp::QoS{1}, std::bind(&VilmaPlatooning::platooning_engage_callback, this, _1),
            sub_options);

        set_distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hmi/set_distance", rclcpp::QoS{1}, std::bind(&VilmaPlatooning::set_distance_callback, this, _1),
            sub_options);

        control_commmand_pub_ = this->create_publisher<autoware_control_msgs::msg::Control>("/control/control_command", rclcpp::QoS{1});
        hmi_target_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/target_speed", rclcpp::QoS{1});
        hmi_follower_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/follower_speed", rclcpp::QoS{1});
        hmi_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hmi/distance", rclcpp::QoS{1});
        hmi_status_pub_ = this->create_publisher<std_msgs::msg::String>("/hmi/status", rclcpp::QoS{1});

        distance_setpoint_.store(10.0);
    }

    void VilmaPlatooning::set_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        distance_setpoint_.store(msg->data);
    }

    void VilmaPlatooning::follower_gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Using %s GNSS.", this->get_parameter("gnss_topic").as_string().c_str());

        following_vehicle_states_mutex_.lock();
        following_vehicle_states_.longitude = msg->longitude;
        following_vehicle_states_.latitude = msg->latitude;
        following_vehicle_states_mutex_.unlock();
    }

    void VilmaPlatooning::control_mode_callback(const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg)
    {
        vehicle_control_mode_.store(msg->mode);
    }

    void VilmaPlatooning::velocity_report_callback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
    {
        following_vehicle_states_mutex_.lock();
        following_vehicle_states_.speed = msg->longitudinal_velocity;
        following_vehicle_states_mutex_.unlock();
    }

    void VilmaPlatooning::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
    {

        RCLCPP_INFO(this->get_logger(), "CAM received");

        sensor_msgs::msg::NavSatFix leader_fix;

        leader_fix.latitude = etsi_its_cam_msgs::access::getLatitude(*msg);
        leader_fix.longitude = etsi_its_cam_msgs::access::getLongitude(*msg);
        leader_fix.altitude = etsi_its_cam_msgs::access::getAltitude(*msg);
        leader_fix.header.frame_id = std::string("leader");
        leader_fix.header.stamp = this->get_clock()->now();

        leader_gnss_pub_->publish(leader_fix);

        leader_vehicle_states_mutex_.lock();

        leader_vehicle_states_.longitude = etsi_its_cam_msgs::access::getLongitude(*msg);
        leader_vehicle_states_.latitude = etsi_its_cam_msgs::access::getLatitude(*msg);
        leader_vehicle_states_.heading = etsi_its_cam_msgs::access::getHeading(*msg);
        leader_vehicle_states_.speed = etsi_its_cam_msgs::access::getSpeed(*msg);
        leader_vehicle_states_.acceleration = etsi_its_cam_msgs::access::getLongitudinalAcceleration(*msg);

        following_vehicle_states_mutex_.lock();

        // Compute distance between vehicles
        getDistance(leader_vehicle_states_, following_vehicle_states_);

        RCLCPP_INFO(this->get_logger(), "Distance: %lf", leader_vehicle_states_.distance);

        following_vehicle_states_mutex_.unlock();
        leader_vehicle_states_mutex_.unlock();
    }

    void VilmaPlatooning::platooning_engage_callback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        bool change_control_mode_result;

        // If the state is the same, do nothing
        if (msg->data == platooning_state_.load())
        {
            return;
        }

        switch (msg->data)
        {
        case VilmaPlatooning::PLATOONING_ENABLE:

            platooning_state_.store(VilmaPlatooning::PLATOONING_ENABLE);

            change_control_mode_result = change_control_mode(ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY);
            break;

        case VilmaPlatooning::PLATOONING_DISABLE:

            platooning_state_.store(VilmaPlatooning::PLATOONING_DISABLE);

            change_control_mode_result = change_control_mode(ControlModeCommand::Request::MANUAL);
            break;

        default:

            platooning_state_.store(VilmaPlatooning::PLATOONING_DISABLE);

            change_control_mode_result = change_control_mode(ControlModeCommand::Request::MANUAL);

            RCLCPP_WARN(this->get_logger(), "Unknow platooning engage command");
            break;
        }

        (void)change_control_mode_result;
    }

    bool VilmaPlatooning::change_control_mode(const uint8_t control_mode)
    {
        auto change_control_mode_request = std::make_shared<ControlModeCommand::Request>();

        uint8_t i = 0;

        while (!control_mode_command_cli_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            }
            RCLCPP_INFO(this->get_logger(), "Service not available yet, waiting again...");

            if (i > 5)
            {
                RCLCPP_ERROR(this->get_logger(), "Can't get response from service.");
                return 0;
            }
            i++;
        }

        change_control_mode_request->mode = control_mode;

        auto change_control_mode_result = control_mode_command_cli_->async_send_request(change_control_mode_request);

        if (change_control_mode_result.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Control mode changed successful.");
            return 1;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Control mode changed unsuccessful.");
            return 0;
        }
    }

    void VilmaPlatooning::getDistance(vehicle_states_t &leader_vehicle, vehicle_states_t &following_vehicle)
    {
        const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();

        geod.Inverse(leader_vehicle.latitude,
                     leader_vehicle.longitude,
                     following_vehicle.latitude,
                     following_vehicle.longitude,
                     leader_vehicle.distance);

        int zone;
        bool is_north;

        GeographicLib::UTMUPS::Forward(leader_vehicle.latitude,
                                       leader_vehicle.longitude,
                                       zone,
                                       is_north,
                                       leader_vehicle.utm_x,
                                       leader_vehicle.utm_y);

        GeographicLib::UTMUPS::Forward(following_vehicle.latitude,
                                       following_vehicle.longitude,
                                       zone,
                                       is_north,
                                       following_vehicle.utm_x,
                                       following_vehicle.utm_y);

        leader_vehicle.lateral_distance = 0.0;      // TODO
        leader_vehicle.longitudinal_distance = 0.0; // TODO

        return;
    }

    void VilmaPlatooning::hmi_update()
    {
        std_msgs::msg::Float32 follower_speed_hmi;

        following_vehicle_states_mutex_.lock();
        follower_speed_hmi.data = following_vehicle_states_.speed;
        following_vehicle_states_mutex_.unlock();

        hmi_follower_speed_pub_->publish(follower_speed_hmi);

        std_msgs::msg::Float32 target_speed_hmi;
        std_msgs::msg::Float32 distance_hmi;

        leader_vehicle_states_mutex_.lock();
        distance_hmi.data = leader_vehicle_states_.distance;
        leader_vehicle_states_mutex_.unlock();

        hmi_distance_pub_->publish(distance_hmi);

        std_msgs::msg::String status_hmi;

        std::string platooning_status;

        switch (platooning_state_.load())
        {
        case VilmaPlatooning::PLATOONING_DISABLE:
            platooning_status = "Platooning disabled";
            break;
        case VilmaPlatooning::PLATOONING_ENABLE:
            platooning_status = "Platooning enabled";
            break;

        default:
            platooning_status = "Platooning error";
            break;
        }

        std::string control_mode_status;

        switch (vehicle_control_mode_.load())
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
        autoware_control_msgs::msg::Control control_action;

        vehicle_states_t follower_states;
        vehicle_states_t target_states;

        following_vehicle_states_mutex_.lock();
        follower_states = following_vehicle_states_;
        following_vehicle_states_mutex_.unlock();

        leader_vehicle_states_mutex_.lock();
        target_states = leader_vehicle_states_;
        leader_vehicle_states_mutex_.unlock();

        // ! Run platooning control

        double error = distance_setpoint_.load() - target_states.distance;

        double kp = 0.3;

        double action = target_states.speed - error * kp; // u = x_dot - e_dist*kp

        control_action.longitudinal.velocity = std::min(0.0, action);
        // control_action.longitudinal.acceleration = target_states.acceleration;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Control \n Error: %lf | Action: %lf", error, action);

        // * Publish desired speed, acceleration, jerk to vehicle in SI units

        if (platooning_state_.load() == VilmaPlatooning::PLATOONING_ENABLE)
        {
            control_commmand_pub_->publish(control_action);
        }
    }

} // namespace VilmaPlatooning
