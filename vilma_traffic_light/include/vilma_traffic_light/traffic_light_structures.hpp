#ifndef vilma_traffic_light_rx__traffic_light_utils_HPP_
#define vilma_traffic_light_rx__traffic_light_utils_HPP_

#include <cstdint>

typedef struct traffic_light
{
  uint8_t state; /// etsi_its_spatem_ts_msgs::msg::MovementPhaseState
  double next_change; /// (ms)
} traffic_light_t;

#endif  // vilma_traffic_light_rx__traffic_light_utils_HPP_
