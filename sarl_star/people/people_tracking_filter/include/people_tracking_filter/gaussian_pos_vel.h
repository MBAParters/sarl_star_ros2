#pragma once

#include <bfl/pdf/pdf.h>
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/gaussian_vector.h"

namespace BFL
{
  /// Class representing gaussian pos_vel
  class GaussianPosVel : public Pdf<StatePosVel>
  {
  private:
    StatePosVel mu_, sigma_;
    GaussianVector gauss_pos_, gauss_vel_;
    mutable double dt_;

  public:
    /// Constructor
    GaussianPosVel(const StatePosVel &mu, const StatePosVel &sigma);

    /// Destructor
    virtual ~GaussianPosVel();

    /// clone function
    virtual GaussianPosVel *Clone() const;

    /// output stream for GaussianPosVel
    friend std::ostream &operator<<(std::ostream &os, const GaussianPosVel &g);

    // set time
    void SetDt(double dt) const
    {
      dt_ = dt;
    };

    // Redefinition of pure virtuals
    virtual Probability ProbabilityGet(const StatePosVel &input) const;
    bool SampleFrom(vector<Sample<StatePosVel>> &list_samples, const int num_samples, int method = DEFAULT, void *args = NULL) const;
    virtual bool SampleFrom(Sample<StatePosVel> &one_sample, int method = DEFAULT, void *args = NULL) const;

    virtual StatePosVel ExpectedValueGet() const;
    virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;
  };

} // end namespace