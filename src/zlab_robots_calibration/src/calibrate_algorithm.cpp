#include "zlab_robots_calibration/calibrate_algorithm.h"

#include <algorithm>
#include <cmath>

CalibrateAlgorithm::CalibrateAlgorithm(int target_count)
  : target_count_(target_count),
    has_last_(false),
    trans_thresh_(0.005),   // 5 mm
    rot_thresh_(1.0 * M_PI / 180.0) // 1 deg
{
}

bool CalibrateAlgorithm::isClose(const tf2::Transform& A,
                                  const tf2::Transform& B) const // 判断该帧和另一帧是否接近，即是否有偶发抖动
{
  double trans_dist = (A.getOrigin() - B.getOrigin()).length();

  tf2::Quaternion qA = A.getRotation();
  tf2::Quaternion qB = B.getRotation();
  double angle = qA.angle(qB);

  return (trans_dist < trans_thresh_) && (angle < rot_thresh_);
}

bool CalibrateAlgorithm::addSample(const tf2::Transform& T1,
                                    const tf2::Transform& T2,
                                    const tf2::Transform& T3)
{
  if (!has_last_)
  {
    last_T1_ = T1;
    last_T2_ = T2;
    last_T3_ = T3;
    has_last_ = true;
    return false;
  }

  if (!isClose(T1, last_T1_) ||
      !isClose(T2, last_T2_) ||
      !isClose(T3, last_T3_))
  {
    return false;  // reject this frame
  }

  buf1_.push_back(T1);
  buf2_.push_back(T2);
  buf3_.push_back(T3);

  last_T1_ = T1;
  last_T2_ = T2;
  last_T3_ = T3;

  return isReady();
}

bool CalibrateAlgorithm::isReady() const
{
  return static_cast<int>(buf1_.size()) >= target_count_;
}

void CalibrateAlgorithm::getMedian(tf2::Transform& T1_out,
                                    tf2::Transform& T2_out,
                                    tf2::Transform& T3_out) const
{
  auto medianTransform = [](const std::vector<tf2::Transform>& buf)
  {
    std::vector<double> xs, ys, zs;
    xs.reserve(buf.size());
    ys.reserve(buf.size());
    zs.reserve(buf.size());

    for (const auto& T : buf)
    {
      xs.push_back(T.getOrigin().x());
      ys.push_back(T.getOrigin().y());
      zs.push_back(T.getOrigin().z());
    }

    auto median = [](std::vector<double>& v)
    {
      size_t n = v.size() / 2;
      std::nth_element(v.begin(), v.begin() + n, v.end());
      return v[n];
    };

    tf2::Vector3 p(median(xs), median(ys), median(zs));

    // rotation: simply take the middle one
    tf2::Quaternion q = buf[buf.size() / 2].getRotation();

    tf2::Transform T;
    T.setOrigin(p);
    T.setRotation(q);
    return T;
  };

  T1_out = medianTransform(buf1_);
  T2_out = medianTransform(buf2_);
  T3_out = medianTransform(buf3_);
}

void CalibrateAlgorithm::reset()
{
  buf1_.clear();
  buf2_.clear();
  buf3_.clear();
  has_last_ = false;
}
