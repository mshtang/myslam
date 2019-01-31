#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include "myslam/common_include.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "myslam/g2o_types.h"

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
    matcher->match(descriptor_ref_, descriptor_curr_, matches);
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
        double d = ref_->findDepth(keypoints_curr_[i]);
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

void VisualOdometry::addKeyFrame()
{
    cout << "Adding a key frame\n";
    map_->insertKeyFrame(curr_);
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = Tcr_estimated_.log();
    Vector3d translation = d.head(3);
    Vector3d rotation = d.tail(3);
    if (translation.norm() > key_frame_min_trans or rotation.norm() > key_frame_min_rot)
        return true;
    return false;
}

bool VisualOdometry::checkEstimationPose()
{
    if (num_inliners_ < min_inliners_)
    {
        cout << "Too few inliners: " << num_inliners_ << ". Rejected!";
        return false;
    }

    Sophus::Vector6d d = Tcr_estimated_.log();
    if (d.norm() > 5)
    {
        cout << "Motion is too large: " << d.norm() << ". Rejected!";
        return false;
    }

    return true;
}

void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (cv::DMatch m : feature_matches_)
    {
        pts3d.push_back(pts_3d_ref_[m.queryIdx]);
        pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
    }

    Mat K = (cv::Mat_<double>(3, 3) << ref_->camera_->fx_, 0, ref_->camera_->cx_,
             0, ref_->camera_->fy_, ref_->camera_->cy_,
             0, 0, 1);
    Mat rvec, tvec, inliners;
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliners);
    num_inliners_ = inliners.rows;
    cout << "pnp inliers: " << num_inliners_ << endl;
    Tcr_estimated_ = SE3(
        Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
        Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
    g2o::SparseOptimizer opt;
    opt.setAlgorithm(solver);

    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Tcr_estimated_.rotation_matrix(), Tcr_estimated_.translation()));
    opt.addVertex(pose);

    for (int i = 0; i < inliners.rows; ++i)
    {
        int index = inliners.at<int>(i, 0);
        EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
        edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        opt.addEdge(edge);
    }

    opt.initializeOptimization(1);
    opt.optimize(10);

    Tcr_estimated_ = SE3(pose->estimate().rotation(), pose->estimate().translation());
}

} // namespace myslam