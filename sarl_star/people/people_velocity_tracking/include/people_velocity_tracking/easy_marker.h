#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

geometry_msgs::msg::Point get_point(const std::vector<double>& position, double scale = 1.0);
geometry_msgs::msg::Quaternion get_quat(const std::vector<double>& orientation);

std_msgs::msg::ColorRGBA get_color(const std::vector<double>& color);

class MarkerGenerator{
public:
    MarkerGenerator();

    void reset();

    visualization_msgs::msg::Marker marker(
        const std::vector<double>& position = {},
        const std::vector<double>& orientation = {},
        const std::vector<std::vector<double>>& points = {},
        const std::vector<std::vector<double>>& colors = {},
        double scale = 1.0,
        const std::vector<double>& color = {});

public:
    int counter_;
    std::string frame_id_;
    std::string ns_;
    int type_;
    int action_;
    std::vector<double> scale_;
    std::vector<double> color_;
    std::vector<std::vector<double>> points_;
    std::vector<std::vector<double>> colors_;
    std::string text_;
    double lifetime_;
};