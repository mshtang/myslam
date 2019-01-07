#include "myslam/frame.h"

namespace myslam
{
Frame::Frame() : id_(-1), time_stamp_(-1) {}
Frame::Frame(long id, double time_stamp, SE3 Tcw, Camera::Ptr camera, Mat color, Mat depth) : id_(id), time_stamp_(time_stamp), Tcw_(Tcw), camera_(camera), color_(color), depth_(depth) {}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++));
}

double Frame::findDepth(const cv::KeyPoint &kp)
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort pt_depth = depth_.ptr<ushort>(y)[x];
    if (pt_depth != 0)
        return double(pt_depth) / camera_->depth_scale_;
    else // if the depth found is 0, search for the neighboring pixel's depth
    {
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; ++i)
        {
            pt_depth = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
            if (pt_depth != 0)
                return double(pt_depth) / camera_->depth_scale_;
        }
    }
    return -1;
}

Vector3d Frame::getCameraCenter() const
{
    return Tcw_.inverse().translation();
}

bool Frame::isInFrame(const Vector3d &pt_world)
{
    Vector3d pc = camera_->world2camera(pt_world, Tcw_);
    if (pc.z() < 0)
        return false;
    Vector2d pp = camera_->camera2pixel(pc);
    return (pp.x() > 0 and pp.x() < color_.cols and pp.y() > 0 and pp.y() < color_.rows);
}
} // namespace myslam