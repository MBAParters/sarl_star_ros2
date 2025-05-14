#pragma once

#include <bfl/pdf/pdf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace BFL
{
/// Class representing gaussian vector
class GaussianVector: public Pdf<tf2::Vector3>
{
private:
  tf2::Vector3 mu_, sigma_;
  mutable double sqrt_;
  mutable tf2::Vector3 sigma_sq_;
  mutable bool sigma_changed_;

public:
  /// Constructor
  GaussianVector(const tf2::Vector3& mu, const tf2::Vector3& sigma);

  /// Destructor
  virtual ~GaussianVector();

  /// output stream for GaussianVector
  friend std::ostream& operator<< (std::ostream& os, const GaussianVector& g);

  void sigmaSet(const tf2::Vector3& sigma);

  // Redefinition of pure virtuals
  virtual Probability ProbabilityGet(const tf2::Vector3& input) const;
  bool SampleFrom(vector<Sample<tf2::Vector3> >& list_samples, const int num_samples, int method = DEFAULT, void * args = NULL) const;
  virtual bool SampleFrom(Sample<tf2::Vector3>& one_sample, int method = DEFAULT, void * args = NULL) const;

  virtual tf2::Vector3 ExpectedValueGet() const;
  virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;

  virtual GaussianVector* Clone() const;
};

} // end namespace