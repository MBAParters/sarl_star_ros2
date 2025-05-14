#pragma once

#include <bfl/pdf/mcpdf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/point_cloud.hpp>

namespace BFL
{
  /// Class representing a vector mcpdf
  class MCPdfVector : public MCPdf<tf2::Vector3>
  {
  public:
    /// Constructor
    MCPdfVector(unsigned int num_samples);

    /// Destructor
    virtual ~MCPdfVector();

    /// Get evenly distributed particle cloud
    void getParticleCloud(const tf2::Vector3 &step, double threshold, sensor_msgs::msg::PointCloud &cloud) const;

    /// Get pos histogram from certain area
    MatrixWrapper::Matrix getHistogram(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;

    virtual tf2::Vector3 ExpectedValueGet() const;
    virtual WeightedSample<tf2::Vector3> SampleGet(unsigned int particle) const;
    virtual unsigned int numParticlesGet() const;
  };

} // end namespace
