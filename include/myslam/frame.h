#pragma once
#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
class Frame
{
  public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_;
    double time_stamp_;
    SE3 Tcw_;
    Camera::Ptr camera_;
    Mat color_, depth_;

    Frame();
    Frame(long id, double time_stamp = 0, SE3 Tcw = SE3(),
          Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
    ~Frame();

    // create a new frame
    static Frame::Ptr createFrame();

    // find the depth of points
    double findDepth(const cv::KeyPoint &kp);

    // get camera center
    Vector3d getCameraCenter() const;

    // check if a point (world) is in the frame
    bool isInFrame(const Vector3d &pt_world);
};
} // namespace myslam