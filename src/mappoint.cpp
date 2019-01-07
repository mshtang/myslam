#include "myslam/mappoint.h"

namespace myslam
{
MapPoint::MapPoint() : id_(-1), pos_(Vector3d(0, 0, 0)), norm_(Vector3d(0, 0, 0)),
                       observed_times_(0), correct_counts_(0) {}
MapPoint::MapPoint(long id, Vector3d pos, Vector3d norm)
    : id_(id), pos_(pos), norm_(norm), observed_times_(0), correct_counts_(0) {}

MapPoint::Ptr MapPoint::createMapPoint()
{
    static long id = 0;
    return MapPoint::Ptr(new MapPoint(id++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
}
} // namespace myslam