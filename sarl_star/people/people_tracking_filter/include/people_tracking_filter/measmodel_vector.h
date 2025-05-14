#pragma once

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

  class MeasPdfVector
      : public BFL::ConditionalPdf<tf2::Vector3, tf2::Vector3>
  {
  public:
    /// Constructor
    MeasPdfVector(const tf2::Vector3 &sigma);

    /// Destructor
    virtual ~MeasPdfVector();

    // set covariance
    void CovarianceSet(const MatrixWrapper::SymmetricMatrix &cov);

    // Redefining pure virtual methods
    virtual BFL::Probability ProbabilityGet(const tf2::Vector3 &input) const;
    virtual bool SampleFrom(BFL::Sample<tf2::Vector3> &one_sample, int method, void *args) const; // Not applicable
    virtual tf2::Vector3 ExpectedValueGet() const;                                                // Not applicable
    virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;                                 // Not applicable

  private:
    GaussianVector meas_noise_;

  }; // class

  class MeasModelVector
      : public BFL::MeasurementModel<tf2::Vector3, tf2::Vector3>
  {
  public:
    /// constructor
    MeasModelVector(const tf2::Vector3 &sigma)
        : BFL::MeasurementModel<tf2::Vector3, tf2::Vector3>(new MeasPdfVector(sigma)) {};

    /// destructor
    ~MeasModelVector()
    {
      delete MeasurementPdfGet();
    };

  }; // class

} // namespace
