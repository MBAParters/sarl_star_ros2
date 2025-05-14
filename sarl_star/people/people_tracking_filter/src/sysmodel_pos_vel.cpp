#include "people_tracking_filter/sysmodel_pos_vel.h"

using namespace std;
using namespace BFL;

static const unsigned int NUM_SYS_POS_VEL_COND_ARGS = 1;
static const unsigned int DIM_SYS_POS_VEL = 6;

// Constructor
SysPdfPosVel::SysPdfPosVel(const StatePosVel &sigma)
    : ConditionalPdf<StatePosVel, StatePosVel>(DIM_SYS_POS_VEL, NUM_SYS_POS_VEL_COND_ARGS),
      noise_(StatePosVel(tf2::Vector3(0, 0, 0), tf2::Vector3(0, 0, 0)), sigma)
{
}

// Destructor
SysPdfPosVel::~SysPdfPosVel()
{
}

Probability
SysPdfPosVel::ProbabilityGet(const StatePosVel &state) const
{
  cerr << "SysPdfPosVel::ProbabilityGet Method not applicable" << endl;
  assert(0);
  return 0;
}

bool SysPdfPosVel::SampleFrom(Sample<StatePosVel> &one_sample, int method, void *args) const
{
  StatePosVel &res = one_sample.ValueGet();

  // get conditional argument: state
  res = this->ConditionalArgumentGet(0);

  // apply system model
  res.pos_ += (res.vel_ * dt_);

  // add noise
  Sample<StatePosVel> noise_sample;
  noise_.SetDt(dt_);
  noise_.SampleFrom(noise_sample, method, args);
  res += noise_sample.ValueGet();

  return true;
}

StatePosVel
SysPdfPosVel::ExpectedValueGet() const
{
  cerr << "SysPdfPosVel::ExpectedValueGet Method not applicable" << endl;
  assert(0);
  return StatePosVel();
}

SymmetricMatrix
SysPdfPosVel::CovarianceGet() const
{
  cerr << "SysPdfPosVel::CovarianceGet Method not applicable" << endl;
  SymmetricMatrix Covar(DIM_SYS_POS_VEL);
  assert(0);
  return Covar;
}
