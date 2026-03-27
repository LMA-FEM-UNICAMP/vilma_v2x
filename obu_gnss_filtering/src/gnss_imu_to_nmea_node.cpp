#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <cmath>
#include <sstream>
#include <iomanip>
#include <chrono>

// UDP
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class NMEAUdpPublisher : public rclcpp::Node
{
public:
  NMEAUdpPublisher()
      : Node("nmea_udp_publisher"),
        lat_(0.0), lon_(0.0), heading_deg_(0.0),
        has_fix_(false)
  {
    // Parameters
    this->declare_parameter<std::string>("obu_ip", "127.0.0.1");
    this->declare_parameter<int>("filtered_gnss_port", 5000);

    udp_ip_ = this->get_parameter("obu_ip").as_string();
    udp_port_ = this->get_parameter("filtered_gnss_port").as_int();

    setupUdp();

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/obu/fix", 10,
        std::bind(&NMEAUdpPublisher::gpsCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/filter/euler", 50,
        std::bind(&NMEAUdpPublisher::imuCallback, this, std::placeholders::_1));

    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", 50,
        std::bind(&NMEAUdpPublisher::canCallback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/twist", 10);
  }

  ~NMEAUdpPublisher()
  {
    close(sock_);
  }

private:
  // ---------------- UDP ----------------

  void setupUdp()
  {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock_ < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket");
      rclcpp::shutdown();
    }

    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(udp_port_);
    servaddr_.sin_addr.s_addr = inet_addr(udp_ip_.c_str());

    RCLCPP_INFO(this->get_logger(),
                "UDP target: %s:%d", udp_ip_.c_str(), udp_port_);
  }

  void sendUdp(const std::string &msg)
  {
    sendto(sock_, msg.c_str(), msg.size(), 0,
           (const struct sockaddr *)&servaddr_,
           sizeof(servaddr_));
  }

  // ---------------- Callbacks ----------------

  void canCallback(const can_msgs::msg::Frame::SharedPtr msg)
  {

    if (msg->id != 0x123)
    {
      return;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;

    twist_msg.header.stamp = this->now();
    twist_msg.header.set__frame_id("base_link");
    twist_msg.twist.twist.linear.x = ((msg->data[0] << 8) + msg->data[1])*100.0; // cm/s to m/s
    twist_pub_->publish(twist_msg);
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    lat_ = msg->latitude;
    lon_ = msg->longitude;
    has_fix_ = true;

    publishNMEA();
  }

  void imuCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    // ENU → NMEA
    double heading = 90.0 - msg->vector.z;
    if (heading < 0.0)
      heading += 360.0;

    heading_deg_ = heading;
  }

  // ---------------- Helpers ----------------

  std::pair<std::string, char> degToNMEA(double deg, bool is_lat)
  {
    double abs_deg = std::abs(deg);
    int d = static_cast<int>(abs_deg);
    double m = (abs_deg - d) * 60.0;

    std::ostringstream oss;

    if (is_lat)
      oss << std::setw(2) << std::setfill('0') << d;
    else
      oss << std::setw(3) << std::setfill('0') << d;

    oss << std::fixed << std::setprecision(4)
        << std::setw(7) << std::setfill('0') << m;

    char dir = (deg >= 0.0) ? (is_lat ? 'N' : 'E') : (is_lat ? 'S' : 'W');

    return {oss.str(), dir};
  }

  std::string checksum(const std::string &s)
  {
    uint8_t cs = 0;
    for (auto c : s)
      cs ^= static_cast<uint8_t>(c);

    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setw(2)
        << std::setfill('0') << static_cast<int>(cs);

    return oss.str();
  }

  void publishNMEA()
  {
    if (!has_fix_)
      return;

    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm *gmt = std::gmtime(&t);

    char time_str[16];
    char date_str[16];

    std::strftime(time_str, sizeof(time_str), "%H%M%S", gmt);
    std::strftime(date_str, sizeof(date_str), "%d%m%y", gmt);

    auto [lat_str, lat_dir] = degToNMEA(lat_, true);
    auto [lon_str, lon_dir] = degToNMEA(lon_, false);

    std::ostringstream body;
    body << "GPRMC,"
         << time_str << ".00,A,"
         << lat_str << "," << lat_dir << ","
         << lon_str << "," << lon_dir << ","
         << "0.00," // speed
         << std::fixed << std::setprecision(2)
         << heading_deg_ << ","
         << date_str
         << ",,,A";

    std::string body_str = body.str();
    std::string full = "$" + body_str + "*" + checksum(body_str) + "\r\n";

    // Send UDP
    sendUdp(full);

    RCLCPP_INFO(this->get_logger(), "%s", full.c_str());
  }

  // ---------------- Members ----------------

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;

  double lat_, lon_;
  double heading_deg_;
  bool has_fix_;

  // UDP
  int sock_;
  struct sockaddr_in servaddr_;
  std::string udp_ip_;
  int udp_port_;
};

// ---------------- Main ----------------

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMEAUdpPublisher>());
  rclcpp::shutdown();
  return 0;
}