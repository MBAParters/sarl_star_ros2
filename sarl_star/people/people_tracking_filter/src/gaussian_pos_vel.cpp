#include "people_tracking_filter/gaussian_pos_vel.h"
#include <bfl/wrappers/rng/rng.h>
#include <cmath>
#include <cassert>

namespace BFL
{
  GaussianPosVel::GaussianPosVel(const StatePosVel &mu, const StatePosVel &sigma)
      : Pdf<StatePosVel>(1),
        mu_(mu),
        sigma_(sigma),
        gauss_pos_(mu.pos_, sigma.pos_),
        gauss_vel_(mu.vel_, sigma.vel_)
  {
  }

  GaussianPosVel::~GaussianPosVel() {}

  GaussianPosVel *GaussianPosVel::Clone() const
  {
    return new GaussianPosVel(mu_, sigma_);
  }

  std::ostream &operator<<(std::ostream &os, const GaussianPosVel &g)
  {
    os << "\nMu pos :\n"
       << g.ExpectedValueGet().pos_ << endl
       << "\nMu vel :\n"
       << g.ExpectedValueGet().vel_ << endl
       << "\nSigma:\n"
       << g.CovarianceGet() << endl;
    return os;
  }

  Probability GaussianPosVel::ProbabilityGet(const StatePosVel &input) const
  {
    return gauss_pos_.ProbabilityGet(input.pos_) * gauss_vel_.ProbabilityGet(input.vel_);
  }

  bool
  GaussianPosVel::SampleFrom(vector<Sample<StatePosVel>> &list_samples, const int num_samples, int method, void *args) const
  {
    list_samples.resize(num_samples);
    vector<Sample<StatePosVel>>::iterator sample_it = list_samples.begin();
    for (sample_it = list_samples.begin(); sample_it != list_samples.end(); sample_it++)
      SampleFrom(*sample_it, method, args);

    return true;
  }

  bool
  GaussianPosVel::SampleFrom(Sample<StatePosVel> &one_sample, int method, void *args) const
  {
    one_sample.ValueSet(StatePosVel(tf2::Vector3(rnorm(mu_.pos_[0], sigma_.pos_[0] * dt_),
                                                 rnorm(mu_.pos_[1], sigma_.pos_[1] * dt_),
                                                 rnorm(mu_.pos_[2], sigma_.pos_[2] * dt_)),
                                    tf2::Vector3(rnorm(mu_.vel_[0], sigma_.vel_[0] * dt_),
                                                 rnorm(mu_.vel_[1], sigma_.vel_[1] * dt_),
                                                 rnorm(mu_.vel_[2], sigma_.vel_[2] * dt_))));
    return true;
  }

  StatePosVel
  GaussianPosVel::ExpectedValueGet() const
  {
    return mu_;
  }

  SymmetricMatrix
  GaussianPosVel::CovarianceGet() const
  {
    SymmetricMatrix sigma(6);
    sigma = 0;
    for (unsigned int i = 0; i < 3; i++)
    {
      sigma(i + 1, i + 1) = pow(sigma_.pos_[i], 2);
      sigma(i + 4, i + 4) = pow(sigma_.vel_[i], 2);
    }
    return sigma;
  }

} // End namespace BFL
