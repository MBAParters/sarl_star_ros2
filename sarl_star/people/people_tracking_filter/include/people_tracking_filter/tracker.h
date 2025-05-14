#pragma once

#include "people_tracking_filter/state_pos_vel.h"
#include <people_msgs/msg/position_measurement.hpp>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <string>

namespace estimation
{

  class Tracker
  {
  public:
    /// constructor
    Tracker(const std::string &name) : name_(name) {};

    /// destructor
    virtual ~Tracker() {};

    /// return the name of the tracker
    const std::string &getName() const
    {
      return name_;
    };

    /// initialize tracker
    virtual void initialize(const BFL::StatePosVel &mu, const BFL::StatePosVel &sigma, const double time) = 0;

    /// return if tracker was initialized
    virtual bool isInitialized() const = 0;

    /// return measure for tracker quality: 0=bad 1=good
    virtual double getQuality() const = 0;

    /// return the lifetime of the tracker
    virtual double getLifetime() const = 0;

    /// return the time of the tracker
    virtual double getTime() const = 0;

    /// update tracker
    virtual bool updatePrediction(const double time) = 0;
    virtual bool updateCorrection(const tf2::Vector3 &meas,
                                  const MatrixWrapper::SymmetricMatrix &cov) = 0;

    /// get filter posterior
    virtual void getEstimate(BFL::StatePosVel &est) const = 0;
    virtual void getEstimate(people_msgs::msg::PositionMeasurement &est) const = 0;

  private:
    std::string name_;

  }; // class

}; // namespace