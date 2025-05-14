#include "people_velocity_tracking/easy_marker.h"

geometry_msgs::msg::Point get_point(const std::vector<double> &position, double scale)
{
    geometry_msgs::msg::Point pt;
    if (position.empty())
    {
        pt.x = 0.0;
        pt.y = 0.0;
        pt.z = 0.0;
    }
    else
    {
        pt.x = position[0] / scale;
        pt.y = position[1] / scale;
        pt.z = position[2] / scale;
    }
    return pt;
}

geometry_msgs::msg::Quaternion get_quat(const std::vector<double> &orientation)
{
    geometry_msgs::msg::Quaternion quat;
    if (orientation.empty())
    {
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = 0.0;
        quat.w = 1.0;
    }
    else if (orientation.size() == 4)
    {
        quat.x = orientation[0];
        quat.y = orientation[1];
        quat.z = orientation[2];
        quat.w = orientation[3];
    }
    else
    {
        tf2::Quaternion q;
        q.setRPY(orientation[0], orientation[1], orientation[2]);
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        quat.w = q.w();
    }
    return quat;
}

std_msgs::msg::ColorRGBA get_color(const std::vector<double> &color)
{
    std_msgs::msg::ColorRGBA rgba;
    if (color.empty())
    {
        rgba.r = 1.0;
        rgba.g = 1.0;
        rgba.b = 1.0;
        rgba.a = 1.0;
    }
    else if (color.size() == 4)
    {
        rgba.r = color[0];
        rgba.g = color[1];
        rgba.b = color[2];
        rgba.a = color[3];
    }
    else
    {
        rgba.r = color[0];
        rgba.g = color[1];
        rgba.b = color[2];
        rgba.a = 1.0;
    }
    return rgba;
}

MarkerGenerator::MarkerGenerator() : counter_(0)
{
    reset();
}

void MarkerGenerator::reset()
{
    counter_ = 0;
    frame_id_ = "";
    ns_ = "marker";
    type_ = visualization_msgs::msg::Marker::ARROW;
    action_ = visualization_msgs::msg::Marker::ADD;
    scale_ = {1.0, 1.0, 1.0};
    color_ = {1.0, 1.0, 1.0, 1.0};
    points_.clear();
    colors_.clear();
    text_ = "";
    lifetime_ = 0.0;
}

visualization_msgs::msg::Marker MarkerGenerator::marker(
    const std::vector<double> &position,
    const std::vector<double> &orientation,
    const std::vector<std::vector<double>> &points,
    const std::vector<std::vector<double>> &colors,
    double scale,
    const std::vector<double> &color)
{
    visualization_msgs::msg::Marker mark;
    mark.header.frame_id = frame_id_;
    mark.ns = ns_;
    mark.type = type_;
    mark.id = counter_;
    mark.action = action_;
    mark.scale.x = scale_[0];
    mark.scale.y = scale_[1];
    mark.scale.z = scale_[2];
    mark.color = color.empty() ? get_color(color_) : get_color(color);
    mark.lifetime = rclcpp::Duration::from_seconds(lifetime_);

    if (!points.empty())
    {
        for (const auto &point : points)
        {
            mark.points.push_back(get_point(point, scale));
        }
    }
    if (!colors.empty())
    {
        for (const auto &col : colors)
        {
            mark.colors.push_back(get_color(col));
        }
    }

    if (!position.empty() || !orientation.empty())
    {
        mark.pose.position = get_point(position, scale);
        mark.pose.orientation = get_quat(orientation);
    }
    else
    {
        mark.pose.orientation.w = 1.0;
    }

    counter_++;
    return mark;
}