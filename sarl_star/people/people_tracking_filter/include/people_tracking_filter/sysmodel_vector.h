#pragma once

#include "people_tracking_filter/gaussian_vector.h"
#include <bfl/model/systemmodel.h>
#include <bfl/pdf/conditionalpdf.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <string>

namespace BFL
{

  class SysPdfVector
      : public ConditionalPdf<tf2::Vector3, tf2::Vector3>
  {
  public:
    /// Constructor
    SysPdfVector(const tf2::Vector3 &sigma);

    /// Destructor
    virtual ~SysPdfVector();

    // set time
    void SetDt(double dt)
    {
      dt_ = dt;
    };

    // Redefining pure virtual methods
    virtual bool SampleFrom(BFL::Sample<tf2::Vector3> &one_sample, int method, void *args) const;
    virtual tf2::Vector3 ExpectedValueGet() const;                       // not applicable
    virtual Probability ProbabilityGet(const tf2::Vector3 &state) const; // not applicable
    virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;       // Not applicable

  private:
    GaussianVector noise_;
    double dt_;

  }; // class

  class SysModelVector
      : public SystemModel<tf2::Vector3>
  {
  public:
    SysModelVector(const tf2::Vector3 &sigma)
        : SystemModel<tf2::Vector3>(new SysPdfVector(sigma)) {};

    /// destructor
    ~SysModelVector()
    {
      delete SystemPdfGet();
    };

    // set time
    void SetDt(double dt)
    {
      ((SysPdfVector *)SystemPdfGet())->SetDt(dt);
    };

  }; // class

} // namespace
