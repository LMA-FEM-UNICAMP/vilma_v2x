#include "vilma_platooning/vilma_platooning.hpp"

namespace vilma_platooning
{
    VilmaPlatooning() : Node("vilma_platooning")
    {

        using std::placeholders::_1;
        cam_sub_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
            "/* topic_name */", 10, std::bind(&VilmaPlatooning::cam_callback, this, _1));
    }

    void VilmaPlatooning::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
    {
        target_vehicle_states_.speed = etsi_its_cam_msgs::access::CAM::getSpeed(msg, speed);
    }
} // namespace VilmaPlatooning
