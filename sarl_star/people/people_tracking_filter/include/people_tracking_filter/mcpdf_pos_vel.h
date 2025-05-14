#pragma once

#include <bfl/pdf/mcpdf.h>
#include "people_tracking_filter/state_pos_vel.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/point_cloud.hpp>

namespace BFL
{
  /// Class representing a posvel mcpdf
  class MCPdfPosVel : public MCPdf<StatePosVel>
  {
  public:
    /// Constructor
    MCPdfPosVel(unsigned int num_samples);

    /// Destructor
    virtual ~MCPdfPosVel();

    /// Get evenly distributed particle cloud
    void getParticleCloud(const tf2::Vector3 &step, double threshold, sensor_msgs::msg::PointCloud &cloud) const;

    /// Get pos histogram from certain area
    MatrixWrapper::Matrix getHistogramPos(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;

    /// Get vel histogram from certain area
    MatrixWrapper::Matrix getHistogramVel(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;

    virtual StatePosVel ExpectedValueGet() const;
    virtual WeightedSample<StatePosVel> SampleGet(unsigned int particle) const;
    virtual unsigned int numParticlesGet() const;

  private:
    /// Get histogram from certain area
    MatrixWrapper::Matrix getHistogram(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step, bool pos_hist) const;
  };

} // end namespace
