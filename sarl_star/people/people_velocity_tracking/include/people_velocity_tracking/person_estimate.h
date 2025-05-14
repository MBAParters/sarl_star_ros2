#pragma once

#include <rclcpp/rclcpp.hpp>
#include <people_msgs/msg/position_measurement_array.hpp>
#include <people_msgs/msg/person.hpp>
#include "people_velocity_tracking/kalman.h"
#include "people_velocity_tracking/easy_marker.h"
#include <visualization_msgs/msg/marker.hpp>

class PersonEstimate
{
public:
    PersonEstimate(const people_msgs::msg::PositionMeasurement &msg);

    void update(const people_msgs::msg::PositionMeasurement &msg);
    rclcpp::Time age() const;
    std::string id() const;
    geometry_msgs::msg::Point velocity() const;
    void publish_markers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, MarkerGenerator &msg);
    std::pair<std::string, people_msgs::msg::Person> get_person();

private:
    geometry_msgs::msg::Point subtract(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) const;
    void scale(geometry_msgs::msg::Point &v, double s) const;
    geometry_msgs::msg::Point add(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &v) const;

    people_msgs::msg::PositionMeasurement pos;
    Kalman k;
    double reliability;
};