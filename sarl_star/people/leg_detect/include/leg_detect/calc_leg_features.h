#pragma once

#include "laser_processor.h"
#include "sensor_msgs/msg/laser_scan.hpp"

std::vector<float> calcLegFeatures(laser_processor::SampleSet* cluster, const sensor_msgs::msg::LaserScan& scan);
