#include "people_velocity_tracking/velocity_tracker.h"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<VelocityTracker> nh = std::make_shared<VelocityTracker>();
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}
