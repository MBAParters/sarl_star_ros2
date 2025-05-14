#pragma once

#include "people_tracking_filter/tracker.h"

// bayesian filtering
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include "people_tracking_filter/state_pos_vel.h"

// TF
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// log files
#include <fstream>

namespace estimation
{

  class TrackerKalman : public Tracker
  {
  public:
    /// constructor
    TrackerKalman(const std::string &name, const BFL::StatePosVel &sysnoise);

    /// destructor
    virtual ~TrackerKalman();

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

  private:
    // pdf / model / filter
    BFL::Gaussian prior_;
    BFL::ExtendedKalmanFilter *filter_;
    BFL::LinearAnalyticConditionalGaussian *sys_pdf_;
    BFL::LinearAnalyticSystemModelGaussianUncertainty *sys_model_;
    BFL::LinearAnalyticConditionalGaussian *meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *meas_model_;
    MatrixWrapper::Matrix sys_matrix_;
    MatrixWrapper::SymmetricMatrix sys_sigma_;

    double calculateQuality();

    // vars
    bool tracker_initialized_;
    double init_time_, filter_time_, quality_;

  }; // class

}; // namespace
