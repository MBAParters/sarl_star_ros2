#include "people_tracking_filter/measmodel_vector.h"

using namespace std;
using namespace BFL;

static const unsigned int NUM_MEASMODEL_VECTOR_COND_ARGS = 1;
static const unsigned int DIM_MEASMODEL_VECTOR = 3;

// Constructor
MeasPdfVector::MeasPdfVector(const tf2::Vector3 &sigma)
    : ConditionalPdf<tf2::Vector3, tf2::Vector3>(DIM_MEASMODEL_VECTOR, NUM_MEASMODEL_VECTOR_COND_ARGS),
      meas_noise_(tf2::Vector3(0, 0, 0), sigma)
{
}

// Destructor
MeasPdfVector::~MeasPdfVector()
{
}

Probability
MeasPdfVector::ProbabilityGet(const tf2::Vector3 &measurement) const
{
  return meas_noise_.ProbabilityGet(measurement - ConditionalArgumentGet(0));
}

bool MeasPdfVector::SampleFrom(Sample<tf2::Vector3> &one_sample, int method, void *args) const
{
  cerr << "MeasPdfVector::SampleFrom Method not applicable" << endl;
  assert(0);
  return false;
}

tf2::Vector3
MeasPdfVector::ExpectedValueGet() const
{
  cerr << "MeasPdfVector::ExpectedValueGet Method not applicable" << endl;
  tf2::Vector3 result;
  assert(0);
  return result;
}

SymmetricMatrix
MeasPdfVector::CovarianceGet() const
{
  cerr << "MeasPdfVector::CovarianceGet Method not applicable" << endl;
  SymmetricMatrix Covar(DIM_MEASMODEL_VECTOR);
  assert(0);
  return Covar;
}

void MeasPdfVector::CovarianceSet(const MatrixWrapper::SymmetricMatrix &cov)
{
  tf2::Vector3 cov_vec(sqrt(cov(1, 1)), sqrt(cov(2, 2)), sqrt(cov(3, 3)));
  meas_noise_.sigmaSet(cov_vec);
}
