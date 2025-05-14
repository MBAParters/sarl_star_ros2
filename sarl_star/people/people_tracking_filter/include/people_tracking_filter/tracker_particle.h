#pragma once

#include "people_tracking_filter/tracker.h"

// bayesian filtering
#include <bfl/filter/bootstrapfilter.h>
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/mcpdf_pos_vel.h"
#include "people_tracking_filter/sysmodel_pos_vel.h"
#include "people_tracking_filter/measmodel_pos.h"

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

  class TrackerParticle : public Tracker
  {
  public:
    /// constructor
    TrackerParticle(const std::string &name, unsigned int num_particles, const BFL::StatePosVel &sysnoise);

    /// destructor
    virtual ~TrackerParticle();

    /// initialize tracker
    virtual void initialize(const BFL::StatePosVel &mu, const BFL::StatePosVel &sigma, const double time);

    /// return if tracker was initialized
    virtual bool isInitialized() const
    {
      return tracker_initialized_;
    };

    /// return measure for tracker quality: 0=bad 1=good
    virtual double getQuality() const
    {
      return quality_;
    };

    /// return the lifetime of the tracker
    virtual double getLifetime() const;

    /// return the time of the tracker
    virtual double getTime() const;

    /// update tracker
    virtual bool updatePrediction(const double time);
    virtual bool updateCorrection(const tf2::Vector3 &meas,
                                  const MatrixWrapper::SymmetricMatrix &cov);

    /// get filter posterior
    virtual void getEstimate(BFL::StatePosVel &est) const;
    virtual void getEstimate(people_msgs::msg::PositionMeasurement &est) const;

    // get evenly spaced particle cloud
    void getParticleCloud(const tf2::Vector3 &step, double threshold, sensor_msgs::msg::PointCloud &cloud) const;

    /// Get histogram from certain area
    MatrixWrapper::Matrix getHistogramPos(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;
    MatrixWrapper::Matrix getHistogramVel(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;

  private:
    // pdf / model / filter
    BFL::MCPdfPosVel prior_;
    BFL::BootstrapFilter<BFL::StatePosVel, tf2::Vector3> *filter_;
    BFL::SysModelPosVel sys_model_;
    BFL::MeasModelPos meas_model_;

    // vars
    bool tracker_initialized_;
    double init_time_, filter_time_, quality_;
    unsigned int num_particles_;

  }; // class

}; // namespace
