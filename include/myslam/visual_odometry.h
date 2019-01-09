#pragma once
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include <opencv2/features2d.hpp>

namespace myslam
{
class VisualOdometry
{
  public:
    typedef std::shared_ptr<VisualOdometry> Ptr;
    enum VOstate
    {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };
    VOstate state_;
    // map with all frames and map points
    Map::Ptr map_;
    // current frame
    Frame::Ptr curr_;
    // reference frame
    Frame::Ptr ref_;

    // orb detector and computing object
    cv::Ptr<cv::ORB> orb_;
    vector<cv::Point3f> pts_3d_ref_;
    vector<cv::KeyPoint> keypoints_curr_;
    Mat descriptor_curr_;
    Mat descriptor_ref_;
    vector<cv::DMatch> feature_matches_;

    // estimated pose of current frame
    SE3 Tcr_estimated_;
    // number of inliner features in ICP
    int num_inliners_;
    // number of lost frames
    int num_lost_;

    // parameters
    int num_of_features_;
    double scale_factor_;
    // number of pyramid levels
    int level_pyramid_;
    // for selecting good matches
    float match_ratio_;
    // max number of lost frames
    int max_num_lost_;
    int min_inliners_;
    // min rotation of two key frames
    double key_frame_min_rot;
    // min translation of two key frames
    double key_frame_min_trans;

  public: // methods
    VisualOdometry();
    ~VisualOdometry();

    // to add a new frame
    bool addFrame(Frame::Ptr frame);

  protected:
    // inner operation
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimationPose();
    bool checkKeyFrame();
};
} // namespace myslam