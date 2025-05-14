#include "people_tracking_filter/sysmodel_vector.h"

using namespace std;
using namespace BFL;

static const unsigned int NUM_SYS_VECTOR_COND_ARGS = 1;
static const unsigned int DIM_SYS_VECTOR = 3;

// Constructor
SysPdfVector::SysPdfVector(const tf2::Vector3 &sigma)
    : ConditionalPdf<tf2::Vector3, tf2::Vector3>(DIM_SYS_VECTOR, NUM_SYS_VECTOR_COND_ARGS),
      noise_(tf2::Vector3(0, 0, 0), sigma)
{
}

// Destructor
SysPdfVector::~SysPdfVector()
{
}

Probability
SysPdfVector::ProbabilityGet(const tf2::Vector3 &state) const
{
  cerr << "SysPdfVector::ProbabilityGet Method not applicable" << endl;
  assert(0);
  return 0;
}

bool SysPdfVector::SampleFrom(Sample<tf2::Vector3> &one_sample, int method, void *args) const
{
  tf2::Vector3 &res = one_sample.ValueGet();

  // get conditional argument: state
  res = this->ConditionalArgumentGet(0);

  // add noise
  Sample<tf2::Vector3> noise_sample;
  noise_.SampleFrom(noise_sample, method, args);
  res += noise_sample.ValueGet();

  return true;
}

tf2::Vector3
SysPdfVector::ExpectedValueGet() const
{
  cerr << "SysPdfVector::ExpectedValueGet Method not applicable" << endl;
  assert(0);
  return tf2::Vector3();
}

SymmetricMatrix
SysPdfVector::CovarianceGet() const
{
  cerr << "SysPdfVector::CovarianceGet Method not applicable" << endl;
  SymmetricMatrix Covar(DIM_SYS_VECTOR);
  assert(0);
  return Covar;
}
