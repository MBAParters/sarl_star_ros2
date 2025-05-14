#include <rclcpp/rclcpp.hpp>

#include "leg_detect/laser_processor.h"
#include "leg_detect/calc_leg_features.h"

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/ml/ml.hpp>

#include <people_msgs/msg/position_measurement.hpp>
#include <people_msgs/msg/position_measurement_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>

#include <message_filters/subscriber.h>

#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/parameter.hpp>

#include <chrono>
#include <algorithm>

using namespace std;
using namespace laser_processor;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

static double no_observation_timeout_s = 0.5;
static double max_second_leg_age_s = 2.0;
static double max_track_jump_m = 1.0;
static double max_meas_jump_m = 0.75; // 1.0
static double leg_pair_separation_m = 1.0;
static string fixed_frame = "map";

static double kal_p = 4, kal_q = .002, kal_r = 10;
static bool use_filter = true;

class SavedFeature
{
public:
  static int nextid;
  std::shared_ptr<tf2_ros::Buffer> tfl_;

  BFL::StatePosVel sys_sigma_;
  TrackerKalman filter_;

  string id_;
  string object_id;
  rclcpp::Time time_;
  rclcpp::Time meas_time_;

  double reliability, p;

  geometry_msgs::msg::PointStamped position_;
  SavedFeature *other;
  float dist_to_person_;

  // one leg tracker
  SavedFeature(geometry_msgs::msg::PointStamped loc, std::shared_ptr<tf2_ros::Buffer> tfl)
      : tfl_(tfl),
        sys_sigma_(tf2::Vector3(0.05, 0.05, 0.05), tf2::Vector3(1.0, 1.0, 1.0)),
        filter_("tracker_name", sys_sigma_),
        reliability(-1.0), p(4)
  {
    char id[100];
    snprintf(id, 100, "legtrack%d", nextid++);
    id_ = std::string(id);

    time_ = loc.header.stamp;
    meas_time_ = loc.header.stamp;
    other = NULL;

    try
    {
      loc = tfl_->transform(loc, fixed_frame);
    }
    catch (tf2::TransformException e)
    {
      RCLCPP_WARN(rclcpp::get_logger("SavedFeature"), "TF exception spot 6.");
    }
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = id_;
    pose.header.stamp = loc.header.stamp;
    pose.child_frame_id = loc.header.frame_id;
    pose.transform.rotation.x = 0.0;
    pose.transform.rotation.y = 0.0;
    pose.transform.rotation.z = 0.0;
    pose.transform.rotation.w = 1.0;
    pose.transform.translation.x = loc.point.x;
    pose.transform.translation.y = loc.point.y;
    pose.transform.translation.z = loc.point.z;
    RCLCPP_WARN(rclcpp::get_logger("SavedFeature"), "%s, %s", id_.c_str(), loc.header.frame_id.c_str());
    // BUG
    // tfl_->setTransform(pose, "default_authority");

    StatePosVel prior_sigma(tf2::Vector3(0.1, 0.1, 0.1), tf2::Vector3(0.0000001, 0.0000001, 0.0000001));
    filter_.initialize(tf2::Vector3(loc.point.x, loc.point.y, loc.point.z), prior_sigma, time_.seconds());

    StatePosVel est;
    filter_.getEstimate(est);
    updatePosition();
  }

  void propagate(rclcpp::Time time)
  {
    time_ = time;

    filter_.updatePrediction(time.seconds());

    updatePosition();
  }

  void update(geometry_msgs::msg::PointStamped loc, double probability)
  {
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = id_;
    pose.header.stamp = loc.header.stamp;
    pose.child_frame_id = loc.header.frame_id;
    pose.transform.rotation.x = 0.0;
    pose.transform.rotation.y = 0.0;
    pose.transform.rotation.z = 0.0;
    pose.transform.rotation.w = 1.0;
    pose.transform.translation.x = loc.point.x;
    pose.transform.translation.y = loc.point.y;
    pose.transform.translation.z = loc.point.z;
    // BUG
    // tfl_->setTransform(pose, "default_authority");

    meas_time_ = loc.header.stamp;
    time_ = meas_time_;

    SymmetricMatrix cov(3);
    cov = 0.0;
    cov(1, 1) = 0.0025;
    cov(2, 2) = 0.0025;
    cov(3, 3) = 0.0025;

    filter_.updateCorrection(tf2::Vector3(loc.point.x, loc.point.y, loc.point.z), cov);

    updatePosition();

    if (reliability < 0 || !use_filter)
    {
      reliability = probability;
      p = kal_p;
    }
    else
    {
      p += kal_q;
      double k = p / (p + kal_r);
      reliability += k * (probability - reliability);
      p *= (1 - k);
    }
  }

  double getLifetime()
  {
    return filter_.getLifetime();
  }

  double getReliability()
  {
    return reliability;
  }

private:
  void updatePosition()
  {
    StatePosVel est;
    filter_.getEstimate(est);

    position_.point.x = est.pos_[0];
    position_.point.y = est.pos_[1];
    position_.point.z = est.pos_[2];
    position_.header.stamp = time_;
    position_.header.frame_id = fixed_frame;
    double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));
    // reliability = fmax(reliability, nreliability);
  }
};

int SavedFeature::nextid = 0;

class MatchedFeature
{
public:
  SampleSet *candidate_;
  SavedFeature *closest_;
  float distance_;
  double probability_;

  MatchedFeature(SampleSet *candidate, SavedFeature *closest, float distance, double probability)
      : candidate_(candidate), closest_(closest), distance_(distance), probability_(probability)
  {
  }

  inline bool operator<(const MatchedFeature &b) const
  {
    return (distance_ < b.distance_);
  }
};

int g_argc;
char **g_argv;

// actual legdetector node
class LegDetector : public rclcpp::Node
{
public:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  ScanMask mask_;

  int mask_count_;

  // cv::ml::RTrees forest;
  cv::Ptr<cv::ml::RTrees> forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  list<SavedFeature *> saved_features_;
  std::mutex saved_mutex_;

  int feature_id_;

  bool use_seeds_;
  bool publish_legs_, publish_people_, publish_leg_markers_, publish_people_markers_;
  int next_p_id_;
  double leg_reliability_limit_;
  int min_points_per_group;

  rclcpp::Publisher<people_msgs::msg::PositionMeasurementArray>::SharedPtr people_measurements_pub_;
  rclcpp::Publisher<people_msgs::msg::PositionMeasurementArray>::SharedPtr leg_measurements_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_pub_;

  message_filters::Subscriber<people_msgs::msg::PositionMeasurement> people_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<people_msgs::msg::PositionMeasurement>> people_notifier_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_notifier_;

  LegDetector() : Node("leg_detector"),
                  mask_count_(0),
                  feat_count_(0),
                  next_p_id_(0)
  {
    kal_p = 4;
    kal_q = 0.02;
    kal_r = 10;
    use_filter = true;

    if (g_argc > 1)
    {
      forest = cv::ml::RTrees::create();
      cv::String feature_file = cv::String(g_argv[1]);
      forest = cv::ml::StatModel::load<cv::ml::RTrees>(feature_file);
      feat_count_ = forest->getVarCount();
      printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
    }
    else
    {
      printf("Please provide a trained random forests classifier as an input.\n");
      rclcpp::shutdown();
    }

    use_seeds_ = declare_parameter("use_seeds", !true);
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);
    people_sub_.subscribe(this, "people_tracker_filter");
    laser_sub_.subscribe(this, "scan_filtered");
    people_notifier_ = std::make_shared<tf2_ros::MessageFilter<people_msgs::msg::PositionMeasurement>>(people_sub_, *buffer_, fixed_frame, 10, get_node_logging_interface(), get_node_clock_interface());
    laser_notifier_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(laser_sub_, *buffer_, fixed_frame, 10, get_node_logging_interface(), get_node_clock_interface());
    markers_pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 20);
    leg_measurements_pub_ = create_publisher<people_msgs::msg::PositionMeasurementArray>("leg_tracker_measurements", 0);
    people_measurements_pub_ = create_publisher<people_msgs::msg::PositionMeasurementArray>("people_tracker_measurements", 0);

    if (use_seeds_)
    {
      people_notifier_->registerCallback(std::bind(&LegDetector::peopleCallback, this, std::placeholders::_1));
      people_notifier_->setTolerance(rclcpp::Duration::from_seconds(0.01));
    }
    laser_notifier_->registerCallback(std::bind(&LegDetector::laserCallback, this, std::placeholders::_1));
    laser_notifier_->setTolerance(rclcpp::Duration::from_seconds(0.01));

    connected_thresh_ = 0.06;
    min_points_per_group = 5;
    leg_reliability_limit_ = 0.7;
    publish_legs_ = true;
    publish_people_ = true;
    publish_leg_markers_ = true;
    publish_people_markers_ = true;

    no_observation_timeout_s = 0.5;
    max_second_leg_age_s = 2.0;
    max_track_jump_m = 1.0;
    max_meas_jump_m = 0.75;
    leg_pair_separation_m = 1.0;
    if (fixed_frame.compare("map") != 0)
    {
      fixed_frame = "map";
      laser_notifier_->setTargetFrame(fixed_frame);
      people_notifier_->setTargetFrame(fixed_frame);
    }

    feature_id_ = 0;
  }

  ~LegDetector()
  {
  }

  double distance(list<SavedFeature *>::iterator it1, list<SavedFeature *>::iterator it2)
  {
    geometry_msgs::msg::PointStamped one = (*it1)->position_, two = (*it2)->position_;
    double dx = one.point.x - two.point.x;
    double dy = one.point.y - two.point.y;
    double dz = one.point.z - two.point.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }

  // Find the tracker that is closest to this person message
  // If a tracker was already assigned to a person, keep this assignment when the distance between them is not too large.
  void peopleCallback(const people_msgs::msg::PositionMeasurement::ConstPtr &people_meas)
  {
    // If there are no legs, return.
    if (saved_features_.empty())
      return;

    geometry_msgs::msg::PointStamped person_loc;
    person_loc.point = people_meas->pos;
    person_loc.header = people_meas->header;
    person_loc.point.z = 0.0;
    geometry_msgs::msg::PointStamped dest_loc = person_loc;

    std::lock_guard<std::mutex> lock(saved_mutex_);
    list<SavedFeature *>::iterator closest = saved_features_.end();
    list<SavedFeature *>::iterator closest1 = saved_features_.end();
    list<SavedFeature *>::iterator closest2 = saved_features_.end();
    float closest_dist = max_meas_jump_m;
    float closest_pair_dist = 2 * max_meas_jump_m;

    list<SavedFeature *>::iterator begin = saved_features_.begin();
    list<SavedFeature *>::iterator end = saved_features_.end();
    list<SavedFeature *>::iterator it1, it2;

    // If there's a pair of legs with the right label and within the max dist, return
    // If there's one leg with the right label and within the max dist,
    //   find a partner for it from the unlabeled legs whose tracks are reasonably new.
    //   If no partners exist, label just the one leg.
    // If there are no legs with the right label and within the max dist,
    //   find a pair of unlabeled legs and assign them the label.
    // If all of the above cases fail,
    //   find a new unlabeled leg and assign the label.

    // For each tracker, get the distance to this person.
    for (it1 = begin; it1 != end; ++it1)
    {
      try
      {
        buffer_->transform(person_loc, dest_loc, (*it1)->id_, tf2_ros::fromMsg(people_meas->header.stamp), fixed_frame);
      }
      catch (...)
      {
        RCLCPP_WARN(get_logger(), "TF exception spot 7.");
      }
      (*it1)->dist_to_person_ = std::hypot(dest_loc.point.x, dest_loc.point.y);
    }

    // Try to find one or two trackers with the same label and within the max distance of the person.
    cout << "Looking for two legs" << endl;
    it2 = end;
    for (it1 = begin; it1 != end; ++it1)
    {
      // If this leg belongs to the person...
      if ((*it1)->object_id == people_meas->object_id)
      {
        // and their distance is close enough...
        if ((*it1)->dist_to_person_ < max_meas_jump_m)
        {
          // if this is the first leg we've found, assign it to it2. Otherwise, leave it assigned to it1 and break.
          if (it2 == end)
            it2 = it1;
          else
            break;
        }
        // Otherwise, remove the tracker's label, it doesn't belong to this person.
        else
        {
          // the two trackers moved apart. This should not happen.
          (*it1)->object_id = "";
        }
      }
    }
    // If we found two legs with the right label and within the max distance, all is good, return.
    if (it1 != end && it2 != end)
    {
      cout << "Found matching pair. The second distance was " << (*it1)->dist_to_person_ << endl;
      return;
    }

    // If we only found one close leg with the right label, let's try to find a second leg that
    //   * doesn't yet have a label  (=valid precondition),
    //   * is within the max distance,
    //   * is less than max_second_leg_age_s old.
    cout << "Looking for one leg plus one new leg" << endl;
    float dist_between_legs, closest_dist_between_legs;
    if (it2 != end)
    {
      closest_dist = max_meas_jump_m;
      closest = saved_features_.end();

      for (it1 = begin; it1 != end; ++it1)
      {
        // Skip this leg track if:
        // - you're already using it.
        // - it already has an id.
        // - it's too old. Old unassigned trackers are unlikely to be the second leg in a pair.
        // - it's too far away from the person.
        if ((it1 == it2) || ((*it1)->object_id != "") || ((*it1)->getLifetime() > max_second_leg_age_s) || ((*it1)->dist_to_person_ >= closest_dist))
          continue;

        // Get the distance between the two legs
        try
        {
          buffer_->transform((*it2)->position_, dest_loc, (*it1)->id_, tf2_ros::fromMsg((*it2)->position_.header.stamp), fixed_frame);
        }
        catch (...)
        {
          RCLCPP_WARN(get_logger(), "TF exception getting distance between legs.");
        }
        dist_between_legs = std::hypot(dest_loc.point.x, dest_loc.point.y);

        // If this is the closest dist (and within range), and the legs are close together and unlabeled, mark it.
        if (dist_between_legs < leg_pair_separation_m)
        {
          closest = it1;
          closest_dist = (*it1)->dist_to_person_;
          closest_dist_between_legs = dist_between_legs;
        }
      }
      // If we found a close, unlabeled leg, set it's label.
      if (closest != end)
      {
        cout << "Replaced one leg with a distance of " << closest_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
        (*closest)->object_id = people_meas->object_id;
      }
      else
      {
        cout << "Returned one matched leg only" << endl;
      }

      // Regardless of whether we found a second leg, return.
      return;
    }

    cout << "Looking for a pair of new legs" << endl;
    // If we didn't find any legs with this person's label, try to find two unlabeled legs that are close together and close to the tracker.
    it1 = saved_features_.begin();
    it2 = saved_features_.begin();
    closest = saved_features_.end();
    closest1 = saved_features_.end();
    closest2 = saved_features_.end();
    closest_dist = max_meas_jump_m;
    closest_pair_dist = 2 * max_meas_jump_m;
    for (; it1 != end; ++it1)
    {
      // Only look at trackers without ids and that are not too far away.
      if ((*it1)->object_id != "" || (*it1)->dist_to_person_ >= max_meas_jump_m)
        continue;

      // Keep the single closest leg around in case none of the pairs work out.
      if ((*it1)->dist_to_person_ < closest_dist)
      {
        closest_dist = (*it1)->dist_to_person_;
        closest = it1;
      }

      // Find a second leg.
      it2 = it1;
      it2++;
      for (; it2 != end; ++it2)
      {
        // Only look at trackers without ids and that are not too far away.
        if ((*it2)->object_id != "" || (*it2)->dist_to_person_ >= max_meas_jump_m)
          continue;

        // Get the distance between the two legs
        try
        {
          buffer_->transform((*it2)->position_, dest_loc, (*it1)->id_, tf2_ros::fromMsg((*it2)->position_.header.stamp), fixed_frame);
        }
        catch (...)
        {
          RCLCPP_WARN(get_logger(), "TF exception getting distance between legs in spot 2.");
        }
        dist_between_legs = std::hypot(dest_loc.point.x, dest_loc.point.y);

        // Ensure that this pair of legs is the closest pair to the tracker, and that the distance between the legs isn't too large.
        if ((*it1)->dist_to_person_ + (*it2)->dist_to_person_ < closest_pair_dist && dist_between_legs < leg_pair_separation_m)
        {
          closest_pair_dist = (*it1)->dist_to_person_ + (*it2)->dist_to_person_;
          closest1 = it1;
          closest2 = it2;
          closest_dist_between_legs = dist_between_legs;
        }
      }
    }
    // Found a pair of legs.
    if (closest1 != end && closest2 != end)
    {
      (*closest1)->object_id = people_meas->object_id;
      (*closest2)->object_id = people_meas->object_id;
      cout << "Found a completely new pair with total distance " << closest_pair_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
      return;
    }

    cout << "Looking for just one leg" << endl;
    // No pair worked, try for just one leg.
    if (closest != end)
    {
      (*closest)->object_id = people_meas->object_id;
      cout << "Returned one new leg only" << endl;
      return;
    }

    cout << "Nothing matched" << endl;
  }

  void pairLegs()
  {
    // Deal With legs that already have ids
    list<SavedFeature *>::iterator begin = saved_features_.begin();
    list<SavedFeature *>::iterator end = saved_features_.end();
    list<SavedFeature *>::iterator leg1, leg2, best, it;

    for (leg1 = begin; leg1 != end; ++leg1)
    {
      // If this leg has no id, skip
      if ((*leg1)->object_id == "")
        continue;

      leg2 = end;
      best = end;
      double closest_dist = leg_pair_separation_m;
      for (it = begin; it != end; ++it)
      {
        if (it == leg1)
          continue;

        if ((*it)->object_id == (*leg1)->object_id)
        {
          leg2 = it;
          break;
        }

        if ((*it)->object_id != "")
          continue;

        double d = distance(it, leg1);
        if (((*it)->getLifetime() <= max_second_leg_age_s) && (d < closest_dist))
        {
          closest_dist = d;
          best = it;
        }
      }

      if (leg2 != end)
      {
        double dist_between_legs = distance(leg1, leg2);
        if (dist_between_legs > leg_pair_separation_m)
        {
          (*leg1)->object_id = "";
          (*leg1)->other = NULL;
          (*leg2)->object_id = "";
          (*leg2)->other = NULL;
        }
        else
        {
          (*leg1)->other = *leg2;
          (*leg2)->other = *leg1;
        }
      }
      else if (best != end)
      {
        (*best)->object_id = (*leg1)->object_id;
        (*leg1)->other = *best;
        (*best)->other = *leg1;
      }
    }

    // Attempt to pair up legs with no id
    for (;;)
    {
      list<SavedFeature *>::iterator best1 = end, best2 = end;
      double closest_dist = leg_pair_separation_m;

      for (leg1 = begin; leg1 != end; ++leg1)
      {
        // If this leg has an id or low reliability, skip
        if ((*leg1)->object_id != "" || (*leg1)->getReliability() < leg_reliability_limit_)
          continue;

        for (leg2 = begin; leg2 != end; ++leg2)
        {
          if (((*leg2)->object_id != "") || ((*leg2)->getReliability() < leg_reliability_limit_) || (leg1 == leg2))
            continue;
          double d = distance(leg1, leg2);
          if (d < closest_dist)
          {
            best1 = leg1;
            best2 = leg2;
          }
        }
      }

      if (best1 != end)
      {
        char id[100];
        snprintf(id, 100, "Person%d", next_p_id_++);
        (*best1)->object_id = std::string(id);
        (*best2)->object_id = std::string(id);
        (*best1)->other = *best2;
        (*best2)->other = *best1;
      }
      else
      {
        break;
      }
    }
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::ConstPtr &scan)
  {
    ScanProcessor processor(*scan, mask_);

    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);
    cv::Mat tmp_mat = cv::Mat(1, feat_count_, CV_32FC1);

    // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
    rclcpp::Time purge = rclcpp::Time(scan->header.stamp) + rclcpp::Duration::from_seconds(-no_observation_timeout_s);
    list<SavedFeature *>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end())
    {
      if ((*sf_iter)->meas_time_ < purge)
      {
        if ((*sf_iter)->other)
          (*sf_iter)->other->other = NULL;
        delete (*sf_iter);
        saved_features_.erase(sf_iter++);
      }
      else
        ++sf_iter;
    }
    // System update of trackers, and copy updated ones in propagate list
    list<SavedFeature *> propagated;
    for (list<SavedFeature *>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++)
    {
      (*sf_iter)->propagate(scan->header.stamp);
      propagated.push_back(*sf_iter);
    }
    // Detection step: build up the set of "candidate" clusters
    // For each candidate, find the closest tracker (within threshold) and add to the match list
    // If no tracker is found, start a new one
    multiset<MatchedFeature> matches;
    for (list<SampleSet *>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      vector<float> f = calcLegFeatures(*i, *scan);

      for (int k = 0; k < feat_count_; k++)
        tmp_mat.data[k] = (float)(f[k]);
      // Probability is the fuzzy measure of the probability that the second element should be chosen,
      // in opencv2 RTrees had a method predict_prob, but that disapeared in opencv3, this is the
      // substitute.
      float probability = 0.5 -
                          forest->predict(tmp_mat, cv::noArray(), cv::ml::RTrees::PREDICT_SUM) /
                              forest->getRoots().size();
      geometry_msgs::msg::PointStamped loc;
      loc.header = scan->header;
      loc.point.x = (*i)->center().getX();
      loc.point.y = (*i)->center().getY();
      loc.point.z = (*i)->center().getZ();
      try
      {
        loc = buffer_->transform(loc, fixed_frame);
      }
      catch (...)
      {
        RCLCPP_WARN(get_logger(), "TF exception spot 3.");
      }
      list<SavedFeature *>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_m;

      for (list<SavedFeature *>::iterator pf_iter = propagated.begin();
           pf_iter != propagated.end();
           pf_iter++)
      {
        // find the closest distance between candidate and trackers
        float dx = loc.point.x - (*pf_iter)->position_.point.x;
        float dy = loc.point.y - (*pf_iter)->position_.point.y;
        float dz = loc.point.z - (*pf_iter)->position_.point.z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < closest_dist)
        {
          closest = pf_iter;
          closest_dist = dist;
        }
      }
      // Nothing close to it, start a new track
      if (closest == propagated.end())
      {
        list<SavedFeature *>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, buffer_));
        RCLCPP_INFO(this->get_logger(), "%d", saved_features_.size());
      }
      // Add the candidate, the tracker and the distance to a match list
      else
      {
        matches.insert(MatchedFeature(*i, *closest, closest_dist, probability));
      }
    }
    // loop through _sorted_ matches list
    // find the match with the shortest distance for each tracker
    while (matches.size() > 0)
    {
      multiset<MatchedFeature>::iterator matched_iter = matches.begin();
      bool found = false;
      list<SavedFeature *>::iterator pf_iter = propagated.begin();
      while (pf_iter != propagated.end())
      {
        // update the tracker with this candidate
        if (matched_iter->closest_ == *pf_iter)
        {
          // Transform candidate to fixed frame
          geometry_msgs::msg::PointStamped loc;
          loc.header = scan->header;
          loc.point.x = matched_iter->candidate_->center().getX();
          loc.point.y = matched_iter->candidate_->center().getY();
          loc.point.z = matched_iter->candidate_->center().getZ();
          try
          {
            loc = buffer_->transform(loc, fixed_frame);
          }
          catch (...)
          {
            RCLCPP_WARN(get_logger(), "TF exception spot 4.");
          }

          // Update the tracker with the candidate location
          matched_iter->closest_->update(loc, matched_iter->probability_);

          // remove this match and
          matches.erase(matched_iter);
          propagated.erase(pf_iter++);
          found = true;
          break;
        }
        // still looking for the tracker to update
        else
        {
          pf_iter++;
        }
      }
      // didn't find tracker to update, because it was deleted above
      // try to assign the candidate to another tracker
      if (!found)
      {
        geometry_msgs::msg::PointStamped loc;
        loc.header = scan->header;
        loc.point.x = matched_iter->candidate_->center().getX();
        loc.point.y = matched_iter->candidate_->center().getY();
        loc.point.z = matched_iter->candidate_->center().getZ();
        try
        {
          loc = buffer_->transform(loc, fixed_frame);
        }
        catch (...)
        {
          RCLCPP_WARN(get_logger(), "TF exception spot 5.");
        }

        list<SavedFeature *>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;

        for (list<SavedFeature *>::iterator remain_iter = propagated.begin();
             remain_iter != propagated.end();
             remain_iter++)
        {
          float dx = loc.point.x - (*remain_iter)->position_.point.x;
          float dy = loc.point.y - (*remain_iter)->position_.point.y;
          float dz = loc.point.z - (*remain_iter)->position_.point.z;
          float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
          if (dist < closest_dist)
          {
            closest = remain_iter;
            closest_dist = dist;
          }
        }

        // no tracker is within a threshold of this candidate
        // so create a new tracker for this candidate
        if (closest == propagated.end())
          list<SavedFeature *>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, buffer_));
        else
          matches.insert(MatchedFeature(matched_iter->candidate_, *closest, closest_dist, matched_iter->probability_));
        matches.erase(matched_iter);
      }
    }
    if (!use_seeds_)
      pairLegs();

    // Publish Data!
    int i = 0;
    vector<people_msgs::msg::PositionMeasurement> people;
    vector<people_msgs::msg::PositionMeasurement> legs;
    for (list<SavedFeature *>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++, i++)
    {
      // reliability
      double reliability = (*sf_iter)->getReliability();

      if ((*sf_iter)->getReliability() > leg_reliability_limit_ && publish_legs_)
      {
        people_msgs::msg::PositionMeasurement pos;
        pos.header.stamp = scan->header.stamp;
        pos.header.frame_id = fixed_frame;
        pos.name = "leg_detector";
        pos.object_id = (*sf_iter)->id_;
        pos.pos = (*sf_iter)->position_.point;
        pos.reliability = reliability;
        pos.covariance[0] = pow(0.3 / reliability, 2.0);
        pos.covariance[1] = 0.0;
        pos.covariance[2] = 0.0;
        pos.covariance[3] = 0.0;
        pos.covariance[4] = pow(0.3 / reliability, 2.0);
        pos.covariance[5] = 0.0;
        pos.covariance[6] = 0.0;
        pos.covariance[7] = 0.0;
        pos.covariance[8] = 10000.0;
        pos.initialization = 0;
        legs.push_back(pos);
      }
      if (publish_leg_markers_)
      {
        visualization_msgs::msg::Marker m;
        m.header.stamp = (*sf_iter)->time_;
        m.header.frame_id = fixed_frame;
        m.ns = "LEGS";
        m.id = i;
        m.type = m.SPHERE;
        m.pose.position = (*sf_iter)->position_.point;
        m.scale.x = 1;
        m.scale.y = 1;
        m.scale.z = 1;
        m.color.a = 1;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        if ((*sf_iter)->object_id != "")
        {
          m.color.r = 1;
        }
        else
        {
          m.color.b = (*sf_iter)->getReliability();
        }
        markers_pub_->publish(m);
      }
      if (publish_people_ || publish_people_markers_)
      {
        SavedFeature *other = (*sf_iter)->other;
        if (other != NULL && other < (*sf_iter))
        {
          double dx = ((*sf_iter)->position_.point.x + other->position_.point.x) / 2,
                 dy = ((*sf_iter)->position_.point.y + other->position_.point.y) / 2,
                 dz = ((*sf_iter)->position_.point.z + other->position_.point.z) / 2;

          if (publish_people_)
          {
            reliability = reliability * other->reliability;
            people_msgs::msg::PositionMeasurement pos;
            pos.header.stamp = now();                              // (*sf_iter)->time_;
            pos.header.frame_id = fixed_frame;
            pos.name = (*sf_iter)->object_id;
            pos.object_id = (*sf_iter)->id_ + "|" + other->id_;
            pos.pos.x = dx;
            pos.pos.y = dy;
            pos.pos.z = dz;
            pos.reliability = reliability;
            pos.covariance[0] = pow(0.3 / reliability, 2.0);
            pos.covariance[1] = 0.0;
            pos.covariance[2] = 0.0;
            pos.covariance[3] = 0.0;
            pos.covariance[4] = pow(0.3 / reliability, 2.0);
            pos.covariance[5] = 0.0;
            pos.covariance[6] = 0.0;
            pos.covariance[7] = 0.0;
            pos.covariance[8] = 10000.0;
            pos.initialization = 0;
            people.push_back(pos);
          }
          if (publish_people_markers_)
          {
            visualization_msgs::msg::Marker m;
            m.header.stamp = (*sf_iter)->time_;
            m.header.frame_id = fixed_frame;
            m.ns = "PEOPLE";
            m.id = i;
            m.type = m.SPHERE;
            m.pose.position.x = dx;
            m.pose.position.y = dy;
            m.pose.position.z = dz;
            m.scale.x = .3;
            m.scale.y = .3;
            m.scale.z = .3;
            m.color.a = 1;
            m.color.b = 1;
            m.lifetime = rclcpp::Duration::from_seconds(0.5);

            markers_pub_->publish(m);
          }
        }
      }
    }
    people_msgs::msg::PositionMeasurementArray array;
    array.header.stamp = now();
    if (publish_legs_)
    {
      array.people = legs;
      leg_measurements_pub_->publish(array);
    }
    if (publish_people_)
    {
      array.people = people;
      people_measurements_pub_->publish(array);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  g_argc = argc;
  g_argv = argv;
  LegDetector::SharedPtr ld = std::make_shared<LegDetector>();
  rclcpp::spin(ld);
  rclcpp::shutdown();
  return 0;
}
