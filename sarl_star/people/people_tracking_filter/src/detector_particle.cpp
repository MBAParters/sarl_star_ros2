#include "people_tracking_filter/detector_particle.h"
#include "people_tracking_filter/uniform_vector.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
using namespace geometry_msgs;

namespace estimation
{
  // constructor
  DetectorParticle::DetectorParticle(unsigned int num_particles) : prior_(num_particles),
                                                                   filter_(NULL),
                                                                   sys_model_(tf2::Vector3(0.1, 0.1, 0.1)),
                                                                   meas_model_(tf2::Vector3(0.1, 0.1, 0.1)),
                                                                   detector_initialized_(false),
                                                                   num_particles_(num_particles)
  {
  }

  // destructor
  DetectorParticle::~DetectorParticle()
  {
    if (filter_)
      delete filter_;
  }

  // initialize prior density of filter
  void DetectorParticle::initialize(const tf2::Vector3 &mu, const tf2::Vector3 &size, const double time)
  {
    cout << "Initializing detector with " << num_particles_ << " particles, with uniform size "
         << size << " around " << mu << endl;

    UniformVector uniform_vector(mu, size);
    vector<Sample<tf2::Vector3>> prior_samples(num_particles_);
    uniform_vector.SampleFrom(prior_samples, num_particles_, CHOLESKY, NULL);
    prior_.ListOfSamplesSet(prior_samples);
    filter_ = new BootstrapFilter<tf2::Vector3, tf2::Vector3>(&prior_, &prior_, 0, num_particles_ / 4.0);

    // detector initialized
    detector_initialized_ = true;
    quality_ = 1;
    filter_time_ = time;
  }

  // update filter prediction
  bool DetectorParticle::updatePrediction(const double dt)
  {
    // set de in sys model
    sys_model_.SetDt(dt);

    // update filter
    bool res = filter_->Update(&sys_model_);
    if (!res)
      quality_ = 0;

    return res;
  }

  // update filter correction
  bool DetectorParticle::updateCorrection(const tf2::Vector3 &meas, const MatrixWrapper::SymmetricMatrix &cov, const double time)
  {
    assert(cov.columns() == 3);

    // set filter time
    filter_time_ = time;

    // set covariance
    ((MeasPdfVector *)(meas_model_.MeasurementPdfGet()))->CovarianceSet(cov);

    // update filter
    bool res = filter_->Update(&meas_model_, meas);
    if (!res)
      quality_ = 0;

    return res;
  }

  // get evenly spaced particle cloud
  void DetectorParticle::getParticleCloud(const tf2::Vector3 &step, double threshold, sensor_msgs::msg::PointCloud &cloud) const
  {
    ((MCPdfVector *)(filter_->PostGet()))->getParticleCloud(step, threshold, cloud);
  }

  // get most recent filter posterior
  void DetectorParticle::getEstimate(tf2::Vector3 &est) const
  {
    est = ((MCPdfVector *)(filter_->PostGet()))->ExpectedValueGet();
  }

  void DetectorParticle::getEstimate(people_msgs::msg::PositionMeasurement &est) const
  {
    tf2::Vector3 tmp = filter_->PostGet()->ExpectedValueGet();

    est.pos.x = tmp[0];
    est.pos.y = tmp[1];
    est.pos.z = tmp[2];

    est.header.stamp = rclcpp::Time(static_cast<uint64_t>(filter_time_ * 1e9));
    est.header.frame_id = "base_link";
  }

  /// Get histogram from certain area
  Matrix DetectorParticle::getHistogram(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const
  {
    return ((MCPdfVector *)(filter_->PostGet()))->getHistogram(min, max, step);
  }

}; // namespace
