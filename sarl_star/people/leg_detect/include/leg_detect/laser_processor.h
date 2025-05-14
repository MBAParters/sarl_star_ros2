#pragma once

#include <unistd.h>
#include <math.h>
#include <list>
#include <set>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace laser_processor
{

  class Sample
  {
  public:
    int index;
    float range;
    float intensity;
    float x;
    float y;

    static Sample *Extract(int ind, const sensor_msgs::msg::LaserScan &scan);

  private:
    Sample() {};
  };

  struct CompareSample
  {
    CompareSample() {}

    inline bool operator()(const Sample* a, const Sample* b) const
    {
      return (a->index < b->index);
    }
  };

  class SampleSet : public std::set<Sample *, CompareSample>
  {
  public:
    SampleSet() {}

    ~SampleSet()
    {
      clear();
    }

    void clear();

    void appendToCloud(sensor_msgs::msg::PointCloud &cloud, int r = 0, int g = 0, int b = 0);

    tf2::Vector3 center();
  };

  //! A mask for filtering out Samples based on range
  class ScanMask
  {
    SampleSet mask_;

    bool filled;
    float angle_min;
    float angle_max;
    uint32_t size;

  public:
    ScanMask() : filled(false), angle_min(0), angle_max(0), size(0) {}

    inline void clear()
    {
      mask_.clear();
      filled = false;
    }

    void addScan(sensor_msgs::msg::LaserScan &scan);

    bool hasSample(Sample *s, float thresh);
  };

  class ScanProcessor
  {
    std::list<SampleSet *> clusters_;
    sensor_msgs::msg::LaserScan scan_;

  public:
    std::list<SampleSet *> &getClusters()
    {
      return clusters_;
    }

    ScanProcessor(const sensor_msgs::msg::LaserScan &scan, ScanMask &mask_, float mask_threshold = 0.3);

    ~ScanProcessor();

    void removeLessThan(uint32_t num);

    void splitConnected(float thresh);
  };
};
