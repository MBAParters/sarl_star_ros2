#pragma once

#include "people_tracking_filter/state_pos_vel.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "people_tracking_filter/gaussian_vector.h"
#include <bfl/model/measurementmodel.h>
#include <bfl/pdf/conditionalpdf.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <string>

namespace BFL
{

  class MeasPdfPos
      : public BFL::ConditionalPdf<tf2::Vector3, StatePosVel>
  {
  public:
    /// Constructor
    MeasPdfPos(const tf2::Vector3 &sigma);

    /// Destructor
    virtual ~MeasPdfPos();

    // set covariance
    void CovarianceSet(const MatrixWrapper::SymmetricMatrix &cov);

    // Redefining pure virtual methods
    virtual BFL::Probability ProbabilityGet(const tf2::Vector3 &input) const;
    virtual bool SampleFrom(BFL::Sample<tf2::Vector3> &one_sample, int method, void *args) const; // Not applicable
    virtual tf2::Vector3 ExpectedValueGet() const;                                                // Not applicable
    virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;                                // Not applicable

  private:
    GaussianVector meas_noise_;

  }; // class

  class MeasModelPos
      : public BFL::MeasurementModel<tf2::Vector3, StatePosVel>
  {
  public:
    /// constructor
    MeasModelPos(const tf2::Vector3 &sigma)
        : BFL::MeasurementModel<tf2::Vector3, StatePosVel>(new MeasPdfPos(sigma)) {};

    /// destructor
    ~MeasModelPos()
    {
      delete MeasurementPdfGet();
    };

  }; // class

} // namespace
