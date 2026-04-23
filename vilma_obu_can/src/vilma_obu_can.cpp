#include "vilma_obu_can/vilma_obu_can.hpp"

#include <limits>

namespace vilma_obu_can
{
VilmaObuCan::VilmaObuCan() : Node("vilma_obu_can_publisher")
{
  this->declare_parameter("can_out", "can0");
  can_out_ = this->get_parameter("can_out").as_string();
  this->declare_parameter("use_free_acc", true);
  bool use_free_acc = this->get_parameter("use_free_acc").as_bool();
  this->declare_parameter("publish_period_ms", 50);
  uint16_t publish_period_ms = this->get_parameter("publish_period_ms").as_int();

  /// OBU CAN communication
  socket_out_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_out_ < 0)
  {
    RCLCPP_FATAL(this->get_logger(), "Error to connect to socket. Closing...");
    this->~VilmaObuCan();
  }
  strcpy(ifr_out_.ifr_name, can_out_.c_str());
  if (ioctl(socket_out_, SIOCGIFINDEX, &ifr_out_) < 0)
  {
    RCLCPP_FATAL(this->get_logger(), "ioctl(SIOCGIFINDEX) failed. Closing...");
    this->~VilmaObuCan();
  }

  addr_out_.can_family = AF_CAN;
  addr_out_.can_ifindex = ifr_out_.ifr_ifindex;
  if (bind(socket_out_, (struct sockaddr*)&addr_out_, sizeof(addr_out_)) < 0)
  {
    RCLCPP_FATAL(this->get_logger(), "bind() failed. Closing...");
    this->~VilmaObuCan();
  }

  /// ROS2

  /* Parameters*/

  using std::placeholders::_1;
  if (!use_free_acc)
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
                                                                std::bind(&VilmaObuCan::imu_callback, this, _1));
  }
  else
  {
    imu_free_acc_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/filter/free_acceleration", 1, std::bind(&VilmaObuCan::free_acc_callback, this, _1));
    imu_angular_vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/imu/angular_velocity", 1, std::bind(&VilmaObuCan::angular_vel_callback, this, _1));
  }

  vilma_velocity_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", 1, std::bind(&VilmaObuCan::vilma_velocity_callback, this, _1));

  send_can_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_period_ms),
                                            std::bind(&VilmaObuCan::send_can_timer_callback, this));

  // Initialization
  is_new_data_ = false;
  vehicle_its_data_ = {};
  vehicle_speed_cm_per_s_ = 0;
}

VilmaObuCan::~VilmaObuCan()
{
  close(socket_out_);
}

int16_t VilmaObuCan::safeAssignToInt16(double value)
{
  if (value > std::numeric_limits<int16_t>::max())
  {
    return std::numeric_limits<int16_t>::max();
  }

  if (value < std::numeric_limits<int16_t>::min())
  {
    return std::numeric_limits<int16_t>::min();
  }

  return static_cast<int16_t>(value);
}

void VilmaObuCan::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  is_new_data_ = true;
  vehicle_its_data_.longitudinal_acceleration = safeAssignToInt16(msg->linear_acceleration.x * 1000);
  vehicle_its_data_.lateral_acceleration = safeAssignToInt16(msg->linear_acceleration.y * 1000);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Longitudinal Acceleration: %d | Lateral Acceleration: %d (mm/s²)",
                       vehicle_its_data_.longitudinal_acceleration, vehicle_its_data_.lateral_acceleration);

  vehicle_its_data_.yaw_rate = safeAssignToInt16(msg->angular_velocity.z * 180.0 / M_PI * 100.0);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Yaw Rate: %d (0.1 deg/s)",
                       vehicle_its_data_.yaw_rate);
}

void VilmaObuCan::angular_vel_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  is_new_data_ = true;
  vehicle_its_data_.yaw_rate = safeAssignToInt16(msg->vector.z * 180.0 / M_PI * 100.0);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Yaw Rate: %d (0.1 deg/s)",
                       vehicle_its_data_.yaw_rate);
}
void VilmaObuCan::free_acc_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  is_new_data_ = true;

  vehicle_its_data_.longitudinal_acceleration = safeAssignToInt16(msg->vector.x * 1000.0);
  vehicle_its_data_.lateral_acceleration = safeAssignToInt16(msg->vector.y * 1000.0);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Longitudinal Acceleration: %d | Lateral Acceleration: %d (mm/s²)",
                       vehicle_its_data_.longitudinal_acceleration, vehicle_its_data_.lateral_acceleration);
}

void VilmaObuCan::vilma_velocity_callback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  vehicle_speed_cm_per_s_ = safeAssignToInt16(msg->longitudinal_velocity * 100.0);
}

void VilmaObuCan::send_can_timer_callback()
{
  if (is_new_data_)
  {
    is_new_data_ = false;

    can_frame_t send_frame;

    //* CAN_IMU -----------------------------------------------------------------------------------------------------

    send_frame.can_id = CAN_IMU;  // IMU data message for Cohda
    send_frame.can_dlc = 8;       // Frame data size

    memset(send_frame.data, 0, sizeof(send_frame.data));  // Setting frame data to 0

    /// Assembling CANVSTATE_IMU packet
    send_frame.data[0] = (vehicle_its_data_.longitudinal_acceleration >> 8) & 0xFF;  // Longitudinal Acceleration MSB
    send_frame.data[1] = vehicle_its_data_.longitudinal_acceleration & 0xFF;         // Longitudinal Acceleration LSB
    send_frame.data[2] = (vehicle_its_data_.lateral_acceleration >> 8) & 0xFF;       // Lateral Acceleration MSB
    send_frame.data[3] = vehicle_its_data_.lateral_acceleration & 0xFF;              // Lateral Acceleration LSB
    send_frame.data[4] = (vehicle_its_data_.yaw_rate >> 8) & 0xFF;                   // Yaw Rate MSB
    send_frame.data[5] = vehicle_its_data_.yaw_rate & 0xFF;                          // Yaw Rate LSB
    send_frame.data[6] = 0x01;                                                       // General Confidence
    send_frame.data[7] = 0x01;                                                       // General Confidence

    int sendbytes = write(socket_out_, &send_frame, sizeof(can_frame_t));

    RCLCPP_DEBUG(this->get_logger(), "CAN data sent; ID: %d; Data length: %d", send_frame.can_id, sendbytes);

    //* CANVSTATE_CANID ----------------------------------------------------------------------------------------------

    send_frame.can_id = CANVSTATE_CANID;  // IMU data message for Cohda
    send_frame.can_dlc = 8;               // Frame data size

    memset(send_frame.data, 0, sizeof(send_frame.data));  // Setting frame data to 0

    /// Assembling CANVSTATE_CANID packet
    send_frame.data[0] = (vehicle_speed_cm_per_s_ >> 8) & 0xFF;  // Vehicle Speed MSB
    send_frame.data[1] = vehicle_speed_cm_per_s_ & 0xFF;         // Vehicle Speed LSB
    send_frame.data[2] = 0x00;                                   // ? Vehicle Speed Confidence
    send_frame.data[3] = 0x00;                                   // ? Detected Lane Position
    send_frame.data[5] = 0x00;                                   // ? Longitudinal Acceleration MSB
    send_frame.data[6] = 0x00;                                   // ? Longitudinal Acceleration LSB
    send_frame.data[7] = 102;  // ? Longitudinal Acceleration confidence (102 - unavailable)

    int sendbytes = write(socket_out_, &send_frame, sizeof(can_frame_t));

    RCLCPP_DEBUG(this->get_logger(), "CAN data sent; ID: %d; Data length: %d", send_frame.can_id, sendbytes);
  }
}

}  // namespace vilma_obu_can
