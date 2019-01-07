#pragma once
#include "myslam/common_include.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"

namespace myslam
{
class Map
{
  public:
    typedef std::shared_ptr<Map> Ptr;
    // all landmarks
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;
    // all keyframes
    unordered_map<unsigned long, Frame::Ptr> keyframes_;

    Map() {}

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);
};
} // namespace myslam