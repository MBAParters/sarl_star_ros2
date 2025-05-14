#include "people_tracking_filter/uniform_vector.h"
#include <bfl/wrappers/rng/rng.h>
#include <cmath>
#include <cassert>

namespace BFL
{
  UniformVector::UniformVector(const tf2::Vector3 &mu, const tf2::Vector3 &size)
      : Pdf<tf2::Vector3>(1),
        mu_(mu),
        size_(size)
  {
    for (unsigned int i = 0; i < 3; i++)
      assert(size_[i] > 0);

    probability_ = 1 / (size_[0] * 2 * size_[1] * 2 * size_[2] * 2);
  }

  UniformVector::~UniformVector() {}

  UniformVector *UniformVector::Clone() const
  {
    return new UniformVector(mu_, size_);
  }

  std::ostream &operator<<(std::ostream &os, const UniformVector &g)
  {
    os << "Mu   :\n"
       << g.ExpectedValueGet() << endl
       << "Size :\n"
       << g.CovarianceGet() << endl;
    return os;
  }

  Probability UniformVector::ProbabilityGet(const tf2::Vector3 &input) const
  {
    for (unsigned int i = 0; i < 3; i++)
    {
      if (input[i] < (mu_[0] - (size_[0])))
        return 0;
      if (input[i] > (mu_[0] + (size_[0])))
        return 0;
    }
    return probability_;
  }

  bool
  UniformVector::SampleFrom(vector<Sample<tf2::Vector3>> &list_samples, const int num_samples, int method, void *args) const
  {
    list_samples.resize(num_samples);
    vector<Sample<tf2::Vector3>>::iterator sample_it = list_samples.begin();
    for (sample_it = list_samples.begin(); sample_it != list_samples.end(); sample_it++)
      SampleFrom(*sample_it, method, args);

    return true;
  }

  bool
  UniformVector::SampleFrom(Sample<tf2::Vector3> &one_sample, int method, void *args) const
  {
    one_sample.ValueSet(tf2::Vector3(((runif() - 0.5) * 2 * size_[0]) + mu_[0],
                                ((runif() - 0.5) * 2 * size_[1]) + mu_[1],
                                ((runif() - 0.5) * 2 * size_[2]) + mu_[2]));
    return true;
  }

  tf2::Vector3
  UniformVector::ExpectedValueGet() const
  {
    return mu_;
  }

  SymmetricMatrix
  UniformVector::CovarianceGet() const
  {
    SymmetricMatrix sigma(3);
    sigma = 0;
    for (unsigned int i = 0; i < 3; i++)
      sigma(i + 1, i + 1) = pow(size_[i], 2);
    return sigma;
  }

} // End namespace BFL
