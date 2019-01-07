#pragma once
#include "myslam/common_include.h"

namespace myslam
{
class MapPoint
{
public:
  typedef std::shared_ptr<MapPoint> Ptr;
  unsigned long id_;
  // world coordinates
  Vector3d pos_;
  // normal of view direction
  Vector3d norm_;
  // descriptor for keypoints
  Mat descriptor_;
  // store the number of observation made by the feature matching algo
  int observed_times_;
  int correct_counts_;

  MapPoint();
  MapPoint(long id, Vector3d pos, Vector3d norm);

  static MapPoint::Ptr createMapPoint();
};
} // namespace myslam