#include "people_tracking_filter/tracker_particle.h"
#include "people_tracking_filter/gaussian_pos_vel.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

namespace estimation
{
  // constructor
  TrackerParticle::TrackerParticle(const string &name, unsigned int num_particles, const StatePosVel &sysnoise) : Tracker(name),
                                                                                                                  prior_(num_particles),
                                                                                                                  filter_(NULL),
                                                                                                                  sys_model_(sysnoise),
                                                                                                                  meas_model_(tf2::Vector3(0.1, 0.1, 0.1)),
                                                                                                                  tracker_initialized_(false),
                                                                                                                  num_particles_(num_particles) {};

  // destructor
  TrackerParticle::~TrackerParticle()
  {
    if (filter_)
      delete filter_;
  };

  // initialize prior density of filter
  void TrackerParticle::initialize(const StatePosVel &mu, const StatePosVel &sigma, const double time)
  {
    cout << "Initializing tracker with " << num_particles_ << " particles, with covariance "
         << sigma << " around " << mu << endl;

    GaussianPosVel gauss_pos_vel(mu, sigma);
    vector<Sample<StatePosVel>> prior_samples(num_particles_);
    gauss_pos_vel.SampleFrom(prior_samples, num_particles_, CHOLESKY, NULL);
    prior_.ListOfSamplesSet(prior_samples);
    filter_ = new BootstrapFilter<StatePosVel, tf2::Vector3>(&prior_, &prior_, 0, num_particles_ / 4.0);

    // tracker initialized
    tracker_initialized_ = true;
    quality_ = 1;
    filter_time_ = time;
    init_time_ = time;
  }

  // update filter prediction
  bool TrackerParticle::updatePrediction(const double time)
  {
    bool res = true;
    if (time > filter_time_)
    {
      // set dt in sys model
      sys_model_.SetDt(time - filter_time_);
      filter_time_ = time;

      // update filter
      res = filter_->Update(&sys_model_);
      if (!res)
        quality_ = 0;
    }
    return res;
  };

  // update filter correction
  bool TrackerParticle::updateCorrection(const tf2::Vector3 &meas, const MatrixWrapper::SymmetricMatrix &cov)
  {
    assert(cov.columns() == 3);

    // set covariance
    ((MeasPdfPos *)(meas_model_.MeasurementPdfGet()))->CovarianceSet(cov);

    // update filter
    bool res = filter_->Update(&meas_model_, meas);
    if (!res)
      quality_ = 0;

    return res;
  };

  // get evenly spaced particle cloud
  void TrackerParticle::getParticleCloud(const tf2::Vector3 &step, double threshold, sensor_msgs::msg::PointCloud &cloud) const
  {
    ((MCPdfPosVel *)(filter_->PostGet()))->getParticleCloud(step, threshold, cloud);
  };

  // get most recent filter posterior
  void TrackerParticle::getEstimate(StatePosVel &est) const
  {
    est = ((MCPdfPosVel *)(filter_->PostGet()))->ExpectedValueGet();
  };

  void TrackerParticle::getEstimate(people_msgs::msg::PositionMeasurement &est) const
  {
    StatePosVel tmp = filter_->PostGet()->ExpectedValueGet();

    est.pos.x = tmp.pos_[0];
    est.pos.y = tmp.pos_[1];
    est.pos.z = tmp.pos_[2];

    est.header.stamp = rclcpp::Time(static_cast<uint64_t>(filter_time_ * 1e9));
    est.object_id = getName();
  }

  /// Get histogram from certain area
  Matrix TrackerParticle::getHistogramPos(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const
  {
    return ((MCPdfPosVel *)(filter_->PostGet()))->getHistogramPos(min, max, step);
  };

  Matrix TrackerParticle::getHistogramVel(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const
  {
    return ((MCPdfPosVel *)(filter_->PostGet()))->getHistogramVel(min, max, step);
  };

  double TrackerParticle::getLifetime() const
  {
    if (tracker_initialized_)
      return filter_time_ - init_time_;
    else
      return 0;
  }

  double TrackerParticle::getTime() const
  {
    if (tracker_initialized_)
      return filter_time_;
    else
      return 0;
  }
}; // namespace
