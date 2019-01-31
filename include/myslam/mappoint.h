#pragma once
#include "myslam/common_include.h"

namespace myslam
{
class MapPoint
{
public:
  typedef std::shared_ptr<MapPoint> Ptr;
  unsigned long id_;
  static unsigned long factory_id_;
  bool good_;

  // world coordinates
  Vector3d pos_;
  // normal of view direction
  Vector3d norm_;
  // descriptor for keypoints
  Mat descriptor_;

  list<Frame *> observed_frames_;
  // store the number of observation made by the feature matching algo
  int matched_times_;
  int visible_times_;

  MapPoint();
  MapPoint(long id, Vector3d &pos, Vector3d &norm, Frame *frame = nullptr, const Mat &descriptor = Mat());

  inline cv::Point3f getPositionCV() const
  {
    return cv::Point3f(pos_.x(), pos_.y(), pos_.z());
  }

  static MapPoint::Ptr createMapPoint();
  static MapPoint::Ptr createMapPoint(const Vector3d &pos_world, const Vector3d &norm_, const Mat &descriptor, Frame *frame);
};
} // namespace myslam