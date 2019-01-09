#include "myslam/visual_odometry.h"
#include "myslam/config.h"

namespace myslam
{
VisualOdometry::VisualOdometry() : state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliners_(0)
{
    num_of_features_ = Config::get<int>("number_of_features");
    scale_factor_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    match_ratio_ = Config::get<double>("match_ratio");
    max_num_lost_ = Config::get<int>("max_num_lost");
    min_inliners_ = Config::get<int>("min_inliner");
    key_frame_min_rot = Config::get<double>("keyframe_rotation");
    key_frame_min_trans = Config::get<double>("keyframe_translation");
    orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
}

VisualOdometry::~VisualOdometry() {}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch (state_)
    {
    case INITIALIZING:
    {
        state_ = OK;
        curr_ = ref_ = frame;
        map_->insertKeyFrame(frame);
        // extract features from first frame
        extractKeyPoints();
        computeDescriptors();
        // compute 3D position of features in ref frame
        setRef3DPoints();
        break;
    }
    case OK:
    {
        curr_ = frame;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        poseEstimationPnP();
        if (checkEstimationPose() == true) // a good estimation
        {
            // Tcw = Tcr * Trw
            curr_->Tcw_ = Tcr_estimated_ * ref_->Tcw_;
            ref_ = curr_;
            setRef3DPoints();
            num_lost_ = 0;
            if (checkKeyFrame() == true)
                addKeyFrame();
        }
        else // bad estimations
        {
            num_lost_++;
            if (num_lost_ > max_num_lost_)
                state_ = LOST;
            return false;
        }
        break;
    }
    case LOST:
    {
        cout << "VO has lost." << endl;
        break;
    }
    }
    return true;
}

void VisualOdometry::extractKeyPoints()
{
    orb_->detect(curr_->color_, keypoints_curr_);
}

void VisualOdometry::computeDescriptors()
{
    orb_->compute(curr_->color_, keypoints_curr_, descriptor_curr_);
}

void VisualOdometry::featureMatching()
{
    vector<cv::DMatch> matches;
    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    matcher->match(descriptor_curr_, descriptor_ref_, matches);
    float min_dist = std::min_element(matches.begin(), matches.end(),
                                      [](const cv::DMatch &match1, const cv::DMatch &match2) { return match1.distance < match2.distance; })
                         ->distance;
    min_dist = min_dist * match_ratio_ < 30.0f ? 30.0f : min_dist * match_ratio_;
    feature_matches_.clear();
    for (auto m : matches)
    {
        if (m.distance < min_dist * 2)
            feature_matches_.push_back(m);
    }
    cout << "Good matches: " << feature_matches_.size() << endl;
}

void VisualOdometry::setRef3DPoints()
{
    pts_3d_ref_.clear();
    descriptor_ref_ = Mat();
    for (int i = 0; i < keypoints_curr_.size(); ++i)
    {
        double d = curr_->findDepth(keypoints_curr_[i]);
        if (d > 0)
        {
            Vector3d pt_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
                d);
            pts_3d_ref_.push_back(cv::Point3f(pt_cam.x(), pt_cam.y(), pt_cam.z()));
            descriptor_ref_.push_back(descriptor_curr_.row(i));
        }
    }
}

} // namespace myslam