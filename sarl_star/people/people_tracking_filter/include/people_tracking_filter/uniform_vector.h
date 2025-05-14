#pragma once

#include <bfl/pdf/pdf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace BFL
{
  /// Class representing uniform vector
  class UniformVector : public Pdf<tf2::Vector3>
  {
  private:
    tf2::Vector3 mu_, size_;
    double probability_;

  public:
    /// Constructor
    UniformVector(const tf2::Vector3 &mu, const tf2::Vector3 &size);

    /// Destructor
    virtual ~UniformVector();

    /// output stream for UniformVector
    friend std::ostream &operator<<(std::ostream &os, const UniformVector &g);

    // Redefinition of pure virtuals
    virtual UniformVector *Clone() const;

    // Redefinition of pure virtuals
    virtual Probability ProbabilityGet(const tf2::Vector3 &input) const;
    bool SampleFrom(vector<Sample<tf2::Vector3>> &list_samples, const int num_samples, int method = DEFAULT, void *args = NULL) const;
    virtual bool SampleFrom(Sample<tf2::Vector3> &one_sample, int method = DEFAULT, void *args = NULL) const;

    virtual tf2::Vector3 ExpectedValueGet() const;
    virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;
  };

} // end namespace
