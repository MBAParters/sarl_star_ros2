#include "people_velocity_tracking/person_estimate.h"

PersonEstimate::PersonEstimate(const people_msgs::msg::PositionMeasurement &msg)
    : pos(msg), k(), reliability(0.1)
{
}

void PersonEstimate::update(const people_msgs::msg::PositionMeasurement &msg)
{
    auto last = pos;
    pos = msg;
    reliability = std::max(reliability, msg.reliability);
    auto ivel = subtract(pos.pos, last.pos);
    double time = (pos.header.stamp.sec - last.header.stamp.sec) + (pos.header.stamp.nanosec - last.header.stamp.nanosec) * 1e-9;
    scale(ivel, 1.0 / time);
    k.update({ivel.x, ivel.y, ivel.z});
}

rclcpp::Time PersonEstimate::age() const
{
    return pos.header.stamp;
}

std::string PersonEstimate::id() const
{
    return pos.object_id;
}

geometry_msgs::msg::Point PersonEstimate::velocity() const
{
    auto k_vals = k.values();
    geometry_msgs::msg::Point v;
    if (k_vals.empty())
    {
        v.x = 0.0;
        v.y = 0.0;
        v.z = 0.0;
    }
    else
    {
        v.x = k_vals[0];
        v.y = k_vals[1];
        v.z = k_vals[2];
    }
    return v;
}

void PersonEstimate::publish_markers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, MarkerGenerator &mark)
{
    mark.scale_ = std::vector<double>({0.1, 0.3, 0.0});
    mark.color_ = std::vector<double>({1.0, 1.0, 1.0, 1.0});
    geometry_msgs::msg::Point vel = velocity();
    std::vector<std::vector<double>> points;
    geometry_msgs::msg::Point p = add(pos.pos, vel);
    points.emplace_back(std::vector<double>({pos.pos.x, pos.pos.y, pos.pos.z}));
    points.emplace_back(std::vector<double>({p.x, p.y, p.z}));
    auto m = mark.marker({}, {}, points);
    m.header = pos.header;
    pub->publish(m);
}

std::pair<std::string, people_msgs::msg::Person> PersonEstimate::get_person()
{
    people_msgs::msg::Person p;
    p.name = id();
    p.position = pos.pos;
    p.velocity = velocity();
    p.reliability = reliability;
    return std::pair<std::string, people_msgs::msg::Person>(pos.header.frame_id, p);
}

geometry_msgs::msg::Point PersonEstimate::subtract(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) const
{
    geometry_msgs::msg::Point result;
    result.x = p1.x - p2.x;
    result.y = p1.y - p2.y;
    result.z = p1.z - p2.z;
    return result;
}

void PersonEstimate::scale(geometry_msgs::msg::Point &v, double s) const
{
    v.x *= s;
    v.y *= s;
    v.z *= s;
}

geometry_msgs::msg::Point PersonEstimate::add(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &v) const
{
    geometry_msgs::msg::Point result;
    result.x = p1.x + v.x;
    result.y = p1.y + v.y;
    result.z = p1.z + v.z;
    return result;
}
