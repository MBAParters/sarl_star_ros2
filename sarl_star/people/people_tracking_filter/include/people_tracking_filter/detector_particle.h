#pragma once

#include "people_tracking_filter/tracker.h"

// bayesian filtering
#include <bfl/filter/bootstrapfilter.h>
#include "people_tracking_filter/mcpdf_vector.h"
#include "people_tracking_filter/measmodel_vector.h"
#include "people_tracking_filter/sysmodel_vector.h"

// TF
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// msgs
#include <sensor_msgs/msg/point_cloud.hpp>

// log files
#include <fstream>

namespace estimation
{

  class DetectorParticle
  {
  public:
    /// constructor
    DetectorParticle(unsigned int num_particles);

    /// destructor
    ~DetectorParticle();

    /// initialize detector
    void initialize(const tf2::Vector3 &mu, const tf2::Vector3 &size, const double time);

    /// return if detector was initialized
    bool isInitialized() const
    {
      return detector_initialized_;
    };

    /// return measure for detector quality: 0=bad 1=good
    double getQuality() const
    {
      return quality_;
    };

    /// update detector
    bool updatePrediction(const double dt);
    bool updateCorrection(const tf2::Vector3 &meas,
                          const MatrixWrapper::SymmetricMatrix &cov,
                          const double time);

    /// get filter posterior
    void getEstimate(tf2::Vector3 &est) const;
    void getEstimate(people_msgs::msg::PositionMeasurement &est) const;

    // get evenly spaced particle cloud
    void getParticleCloud(const tf2::Vector3 &step, double threshold, sensor_msgs::msg::PointCloud &cloud) const;

    /// Get histogram from certain area
    MatrixWrapper::Matrix getHistogram(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;

  private:
    // pdf / model / filter
    BFL::MCPdfVector prior_;
    BFL::BootstrapFilter<tf2::Vector3, tf2::Vector3> *filter_;
    BFL::SysModelVector sys_model_;
    BFL::MeasModelVector meas_model_;

    // vars
    bool detector_initialized_;
    double filter_time_, quality_;
    unsigned int num_particles_;

  }; // class

}; // namespace