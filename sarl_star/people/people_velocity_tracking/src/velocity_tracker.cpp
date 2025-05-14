#include "people_velocity_tracking/velocity_tracker.h"

VelocityTracker::VelocityTracker() : Node("velocity_tracker")
{
    mark.type_ = visualization_msgs::msg::Marker::ARROW;
    mark.ns_ = "people_velocities";
    mark.lifetime_ = 0.5;

    timeout_ = this->declare_parameter("timeout", 1.0);
    sub_ = this->create_subscription<people_msgs::msg::PositionMeasurementArray>(
        "/people_tracker_measurements", 10,
        std::bind(&VelocityTracker::pm_cb, this, std::placeholders::_1));
    mpub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    ppub_ = this->create_publisher<people_msgs::msg::People>("/people", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&VelocityTracker::spin, this));
}

void VelocityTracker::pm_cb(const people_msgs::msg::PositionMeasurementArray::SharedPtr msg)
{
    for (const auto &pm : msg->people)
    {
        if (people_.find(pm.object_id) != people_.end())
        {
            people_[pm.object_id]->update(pm);
        }
        else
        {
            people_[pm.object_id] = std::make_shared<PersonEstimate>(pm);
        }
    }
}

void VelocityTracker::spin()
{
    auto now = this->now();
    for (auto it = people_.begin(); it != people_.end();)
    {
        if ((now - it->second->age()) .seconds() > timeout_)
        {
            it = people_.erase(it);
        }
        else
        {
            ++it;
        }
    }
    publish();
}

void VelocityTracker::publish()
{
    mark.counter_ = 0;
    people_msgs::msg::People pl;

    for (const auto &[id, person] : people_)
    {
        person->publish_markers(mpub_, mark);
        auto [frame, p] = person->get_person();
        pl.header.frame_id = frame;
        pl.people.push_back(p);
    }
    ppub_->publish(pl);
}