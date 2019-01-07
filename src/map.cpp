#include "myslam/map.h"

namespace myslam
{
void Map::insertKeyFrame(Frame::Ptr frame)
{
    cout << "Key frame's size = " << keyframes_.size() << endl;
    if (keyframes_.find(frame->id_) == keyframes_.end()) // frame's id not found
    {
        keyframes_.insert(make_pair(frame->id_, frame));
    }
    else // if already exists, just update
    {
        keyframes_[frame->id_] = frame;
    }
}

void Map::insertMapPoint(MapPoint::Ptr map_point)
{
    cout << "Map point's size = " << map_points_.size() << endl;
    if (map_points_.find(map_point->id_) == map_points_.end())
    {
        map_points_.insert(make_pair(map_point->id_, map_point));
    }
    else
    {
        map_points_[map_point->id_] = map_point;
    }
}
} // namespace myslam