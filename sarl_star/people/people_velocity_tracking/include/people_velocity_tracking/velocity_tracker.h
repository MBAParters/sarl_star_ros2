#pragma once

#include <rclcpp/rclcpp.hpp>
#include <people_msgs/msg/position_measurement_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <people_msgs/msg/people.hpp>
#include "people_velocity_tracking/person_estimate.h"
#include "people_velocity_tracking/easy_marker.h"

using namespace std::chrono;

class VelocityTracker : public rclcpp::Node {
public:
    VelocityTracker();

private:
    void pm_cb(const people_msgs::msg::PositionMeasurementArray::SharedPtr msg);
    void spin();
    void publish();

    double timeout_;
    std::unordered_map<std::string, std::shared_ptr<PersonEstimate>> people_;
    rclcpp::Subscription<people_msgs::msg::PositionMeasurementArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpub_;
    rclcpp::Publisher<people_msgs::msg::People>::SharedPtr ppub_;
    rclcpp::TimerBase::SharedPtr timer_;
    MarkerGenerator mark;
};