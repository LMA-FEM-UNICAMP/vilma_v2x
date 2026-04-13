#include "vilma_traffic_light/vilma_traffic_light_rx.hpp"

namespace vilma_traffic_light_rx
{
VilmaTrafficLightRx::VilmaTrafficLightRx(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("vilma_traffic_light_rx", node_options)
{
  RCLCPP_INFO(this->get_logger(), "Starting VilmaTrafficLightRx class...");

  using std::placeholders::_1;
  spatem_sub_ =
      this->create_subscription<SPATEM>("/spatem", 10, std::bind(&VilmaTrafficLightRx::spatem_callback, this, _1));
  mapem_sub_ =
      this->create_subscription<MAPEM>("/mapem", 10, std::bind(&VilmaTrafficLightRx::mapem_callback, this, _1));
}

void VilmaTrafficLightRx::spatem_callback(const SPATEM::SharedPtr msg)
{
  RCLCPP_WARN(this->get_logger(), "SPATEM received!");

  /// For each intersection in the message:
  for (auto& intersection : msg->spat.intersections.array)
  {
    //* SPAT information for a single intersection

    /// For each state in the intersection:
    for (auto& state : intersection.states.array)
    {
      //* ~ Traffic light ifself
      //* Convey various information about the current or future movement state of a designated collection of one or
      //* more lanes of a common type.
      //* - It is a array containing current and future information.

      /// For each event in the state:
      for (auto& event : state.state_time_speed.array)
      {
        //* Contains details about a single movement

        // TODO: show just current movement

        traffic_light_status_.state = event.event_state.value;

        RCLCPP_INFO(this->get_logger(), "Traffic light status: %s",
                    interpretMovementPhaseStateAsText(traffic_light_status_.state).c_str());

        if (event.timing_is_present)
        {
          etsi_its_spatem_ts_msgs::access::time_mark_value_interpretation time_mark_min_end_time_type =
              etsi_its_spatem_ts_msgs::access::interpretTimeMarkValueType(event.timing.min_end_time.value);

          if (time_mark_min_end_time_type == etsi_its_spatem_ts_msgs::access::time_mark_value_interpretation::normal)
          {
            int64_t next_change_ns = etsi_its_spatem_ts_msgs::access::interpretTimeMarkDeltaTimeAsNanoSeconds(
                event.timing.min_end_time.value, this->now().nanoseconds());

            traffic_light_status_.next_change = next_change_ns * 1e-6;

            RCLCPP_INFO(this->get_logger(), "Time to next phase: %.2f ms", traffic_light_status_.next_change);
          }
        }
      }
    }
  }
}

void VilmaTrafficLightRx::mapem_callback(const MAPEM::SharedPtr msg)
{
  RCLCPP_WARN(this->get_logger(), "MAPEM received!");
  (void)msg;
}

/**
 * @brief Interprets the MovementPhaseState type as a string
 *
 * @param movement_phase_state Encoded color value from msg type MovementPhaseState
 * @return String with traffic light color
 */
std::string VilmaTrafficLightRx::interpretMovementPhaseStateAsText(const uint8_t movement_phase_state)
{
  std::string color;

  switch (movement_phase_state)
  {
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::UNAVAILABLE:
      color = "OFF";
      break;

    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::DARK:
      color = "OFF";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::STOP_THEN_PROCEED:
      color = "RED";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::STOP_AND_REMAIN:
      color = "RED";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::PRE_MOVEMENT:
      color = "YELLOW";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::PERMISSIVE_MOVEMENT_ALLOWED:
      color = "GREEN";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::PROTECTED_MOVEMENT_ALLOWED:
      color = "GREEN";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::PERMISSIVE_CLEARANCE:
      color = "YELLOW";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::PROTECTED_CLEARANCE:
      color = "YELLOW";
      break;
    case etsi_its_spatem_ts_msgs::msg::MovementPhaseState::CAUTION_CONFLICTING_TRAFFIC:
      color = "YELLOW";
      break;
    default:
      color = "OFF";
      break;
  }

  return color;
}

}  // namespace vilma_traffic_light_rx

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vilma_traffic_light_rx::VilmaTrafficLightRx)
