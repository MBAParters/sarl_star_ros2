#include "people_tracking_filter/measmodel_pos.h"

using namespace std;
using namespace BFL;

static const unsigned int NUM_MEASMODEL_POS_COND_ARGS = 1;
static const unsigned int DIM_MEASMODEL_POS = 13;

// Constructor
MeasPdfPos::MeasPdfPos(const tf2::Vector3 &sigma)
    : ConditionalPdf<tf2::Vector3, StatePosVel>(DIM_MEASMODEL_POS, NUM_MEASMODEL_POS_COND_ARGS),
      meas_noise_(tf2::Vector3(0, 0, 0), sigma)
{
}

// Destructor
MeasPdfPos::~MeasPdfPos()
{
}

Probability
MeasPdfPos::ProbabilityGet(const tf2::Vector3 &measurement) const
{
  return meas_noise_.ProbabilityGet(measurement - ConditionalArgumentGet(0).pos_);
}

bool MeasPdfPos::SampleFrom(Sample<tf2::Vector3> &one_sample, int method, void *args) const
{
  cerr << "MeasPdfPos::SampleFrom Method not applicable" << endl;
  assert(0);
  return false;
}

tf2::Vector3
MeasPdfPos::ExpectedValueGet() const
{
  cerr << "MeasPdfPos::ExpectedValueGet Method not applicable" << endl;
  tf2::Vector3 result;
  assert(0);
  return result;
}

SymmetricMatrix
MeasPdfPos::CovarianceGet() const
{
  cerr << "MeasPdfPos::CovarianceGet Method not applicable" << endl;
  SymmetricMatrix Covar(DIM_MEASMODEL_POS);
  assert(0);
  return Covar;
}

void MeasPdfPos::CovarianceSet(const MatrixWrapper::SymmetricMatrix &cov)
{
  tf2::Vector3 cov_vec(sqrt(cov(1, 1)), sqrt(cov(2, 2)), sqrt(cov(3, 3)));
  meas_noise_.sigmaSet(cov_vec);
}
