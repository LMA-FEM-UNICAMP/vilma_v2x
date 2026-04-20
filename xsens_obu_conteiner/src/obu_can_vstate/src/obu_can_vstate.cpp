#include "obu_can_vstate/obu_can_vstate.hpp"

namespace obu_can_vstate
{
ObuCanVState::ObuCanVState() : Node("obu_can_vstate_publisher")
{
  can_out_ = "vcan0";

  this->declare_parameter("can_out", "can0");
  can_out_ = this->get_parameter("can_out").as_string();

  /// OBU CAN communication
  socket_out_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(ifr_out_.ifr_name, can_out_.c_str());
  ioctl(socket_out_, SIOCGIFINDEX, &ifr_out_);

  addr_out_.can_family = AF_CAN;
  addr_out_.can_ifindex = ifr_out_.ifr_ifindex;
  bind(socket_out_, (struct sockaddr*)&addr_out_, sizeof(addr_out_));

  /// ROS2

  /* Parameters*/
  obu_vstate_can_id_ = 0x124;
  bool use_imu_data = true;

  using std::placeholders::_1;
  if (use_imu_data)
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
                                                                std::bind(&ObuCanVState::imu_callback, this, _1));
  }
  else
  {
    imu_free_acc_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/filter/free_acceleration", 10, std::bind(&ObuCanVState::free_acc_callback, this, _1));
  }
  imu_angular_vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/angular_velocity", 10, std::bind(&ObuCanVState::angular_vel_callback, this, _1));

  send_can_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObuCanVState::send_can_timer_callback, this));

  // Initialization
  is_new_data_ = false;
  vehicle_its_data_ = {};
}

ObuCanVState::~ObuCanVState()
{
  close(socket_out_);
}

void ObuCanVState::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  is_new_data_ = true;
  vehicle_its_data_.longitudinal_acceleration = msg->linear_acceleration.x * 1000;
  vehicle_its_data_.lateral_acceleration = msg->linear_acceleration.y * 1000;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                       "Longitudinal Acceleration: %d | Lateral Acceleration: %d (mm/s²)",
                       vehicle_its_data_.longitudinal_acceleration, vehicle_its_data_.lateral_acceleration);
}

void ObuCanVState::angular_vel_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  is_new_data_ = true;
  vehicle_its_data_.yaw_rate = msg->vector.z * 180 / M_PI * 100;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Yaw Rate: %d (0.1 deg/s)",
                       vehicle_its_data_.yaw_rate);
}
void ObuCanVState::free_acc_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  is_new_data_ = true;

  vehicle_its_data_.longitudinal_acceleration = msg->vector.x * 1000;
  vehicle_its_data_.lateral_acceleration = msg->vector.y * 1000;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                       "Longitudinal Acceleration: %d | Lateral Acceleration: %d (mm/s²)",
                       vehicle_its_data_.longitudinal_acceleration, vehicle_its_data_.lateral_acceleration);
}
void ObuCanVState::send_can_timer_callback()
{
  if (is_new_data_)
  {
    is_new_data_ = false;

    can_frame_t send_frame;

    send_frame.can_id = obu_vstate_can_id_;  // Example message from Cohda
    send_frame.can_dlc = 8;                  // Frame data size

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
  }
}

}  // namespace obu_can_vstate
