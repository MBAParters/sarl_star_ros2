#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace BFL
{
  /// Class representing state with pos and vel
  class StatePosVel
  {
  public:
    tf2::Vector3 pos_, vel_;

    /// Constructor
    StatePosVel(const tf2::Vector3 &pos = tf2::Vector3(0, 0, 0),
                const tf2::Vector3 &vel = tf2::Vector3(0, 0, 0)) : pos_(pos), vel_(vel) {};

    /// Destructor
    ~StatePosVel() {};

    /// operator +=
    StatePosVel &operator+=(const StatePosVel &s)
    {
      this->pos_ += s.pos_;
      this->vel_ += s.vel_;
      return *this;
    }

    /// operator +
    StatePosVel operator+(const StatePosVel &s)
    {
      StatePosVel res;

      res.pos_ = this->pos_ + s.pos_;
      res.vel_ = this->vel_ + s.vel_;
      return res;
    }

    /// output stream for StatePosVel
    friend std::ostream &operator<<(std::ostream &os, const StatePosVel &s)
    {
      os << "(" << s.pos_[0] << ", " << s.pos_[1] << ", " << s.pos_[2] << ")--("
         << "(" << s.vel_[0] << ", " << s.vel_[1] << ", " << s.vel_[2] << ") ";
      return os;
    };
  };
} // end namespace