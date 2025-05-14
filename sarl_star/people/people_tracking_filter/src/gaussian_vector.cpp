#include "people_tracking_filter/gaussian_vector.h"
#include <bfl/wrappers/rng/rng.h>
#include <cmath>
#include <cassert>

namespace BFL
{
  GaussianVector::GaussianVector(const tf2::Vector3 &mu, const tf2::Vector3 &sigma)
      : Pdf<tf2::Vector3>(1),
        mu_(mu),
        sigma_(sigma),
        sigma_changed_(true)
  {
    for (unsigned int i = 0; i < 3; i++)
      assert(sigma[i] > 0);
  }

  GaussianVector::~GaussianVector() {}

  std::ostream &operator<<(std::ostream &os, const GaussianVector &g)
  {
    os << "Mu   :\n"
       << g.ExpectedValueGet() << endl
       << "Sigma:\n"
       << g.CovarianceGet() << endl;
    return os;
  }

  void GaussianVector::sigmaSet(const tf2::Vector3 &sigma)
  {
    sigma_ = sigma;
    sigma_changed_ = true;
  }

  Probability GaussianVector::ProbabilityGet(const tf2::Vector3 &input) const
  {
    if (sigma_changed_)
    {
      sigma_changed_ = false;
      // 2 * sigma^2
      for (unsigned int i = 0; i < 3; i++)
        sigma_sq_[i] = 2 * sigma_[i] * sigma_[i];
      // sqrt
      sqrt_ = 1 / sqrt(M_PI * M_PI * M_PI * sigma_sq_[0] * sigma_sq_[1] * sigma_sq_[2]);
    }

    tf2::Vector3 diff = input - mu_;
    return sqrt_ * exp(-(diff[0] * diff[0] / sigma_sq_[0]) - (diff[1] * diff[1] / sigma_sq_[1]) - (diff[2] * diff[2] / sigma_sq_[2]));
  }

  bool
  GaussianVector::SampleFrom(vector<Sample<tf2::Vector3>> &list_samples, const int num_samples, int method, void *args) const
  {
    list_samples.resize(num_samples);
    vector<Sample<tf2::Vector3>>::iterator sample_it = list_samples.begin();
    for (sample_it = list_samples.begin(); sample_it != list_samples.end(); sample_it++)
      SampleFrom(*sample_it, method, args);

    return true;
  }

  bool
  GaussianVector::SampleFrom(Sample<tf2::Vector3> &one_sample, int method, void *args) const
  {
    one_sample.ValueSet(tf2::Vector3(rnorm(mu_[0], sigma_[0]),
                                     rnorm(mu_[1], sigma_[1]),
                                     rnorm(mu_[2], sigma_[2])));
    return true;
  }

  tf2::Vector3
  GaussianVector::ExpectedValueGet() const
  {
    return mu_;
  }

  SymmetricMatrix
  GaussianVector::CovarianceGet() const
  {
    SymmetricMatrix sigma(3);
    sigma = 0;
    for (unsigned int i = 0; i < 3; i++)
      sigma(i + 1, i + 1) = pow(sigma_[i], 2);
    return sigma;
  }

  GaussianVector *
  GaussianVector::Clone() const
  {
    return new GaussianVector(mu_, sigma_);
  }

} // End namespace BFL
