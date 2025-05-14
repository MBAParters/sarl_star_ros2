#pragma once

#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/gaussian_pos_vel.h"
#include <bfl/model/systemmodel.h>
#include <bfl/pdf/conditionalpdf.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <string>

namespace BFL
{

  class SysPdfPosVel
      : public ConditionalPdf<StatePosVel, StatePosVel>
  {
  public:
    /// Constructor
    SysPdfPosVel(const StatePosVel &sigma);

    /// Destructor
    virtual ~SysPdfPosVel();

    // set time
    void SetDt(double dt)
    {
      dt_ = dt;
    };

    // Redefining pure virtual methods
    virtual bool SampleFrom(BFL::Sample<StatePosVel> &one_sample, int method, void *args) const;
    virtual StatePosVel ExpectedValueGet() const;                       // not applicable
    virtual Probability ProbabilityGet(const StatePosVel &state) const; // not applicable
    virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;       // Not applicable

  private:
    GaussianPosVel noise_;
    double dt_;

  }; // class

  class SysModelPosVel
      : public SystemModel<StatePosVel>
  {
  public:
    SysModelPosVel(const StatePosVel &sigma)
        : SystemModel<StatePosVel>(new SysPdfPosVel(sigma)) {};

    /// destructor
    ~SysModelPosVel()
    {
      delete SystemPdfGet();
    };

    // set time
    void SetDt(double dt)
    {
      ((SysPdfPosVel *)SystemPdfGet())->SetDt(dt);
    };

  }; // class

} // namespace
