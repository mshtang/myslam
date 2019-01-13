#include <fstream>
#include "myslam/common_include.h"
#include "myslam/camera.h"
#include "myslam/config.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"
#include "myslam/visual_odometry.h"
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <boost/timer.hpp>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "usage: run_vo parameter_file" << endl;
        return 1;
    }

    myslam::Config::setParamFile(argv[1]);
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);

    string dataset_dir = myslam::Config::get<string>("dataset_dir");
    cout << "dataset: " << dataset_dir << endl;
    std::ifstream fin(dataset_dir + "/associated.txt");
    if (!fin)
    {
        cout << "Input the associated txt file!" << endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while (!fin.eof())
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
        rgb_times.push_back(atof(rgb_time.c_str()));
        rgb_files.push_back(dataset_dir + "/" + rgb_file);
        depth_times.push_back(atof(depth_time.c_str()));
        depth_files.push_back(dataset_dir + "/" + depth_file);

        if (fin.good() == false)
            break;
    }
    myslam::Camera::Ptr camera(new myslam::Camera);

    // viz
    cv::viz::Viz3d vis("visual odometry");
    cv::viz::WCoordinateSystem world_coord(1.0), camera_coord(0.5);
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);
    world_coord.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coord.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("world", world_coord);
    vis.showWidget("camera", camera_coord);

    cout << "read total " << rgb_files.size() << " entries\n";
    for (int i = 0; i < rgb_files.size(); ++i)
    {
        Mat color = cv::imread(rgb_files[i]);
        Mat depth = cv::imread(depth_files[i], -1);
        if (color.data == nullptr or depth.data == nullptr)
            break;
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame(pFrame);
        cout << "VO costs time: " << timer.elapsed() << endl;

        if (vo->state_ == myslam::VisualOdometry::LOST)
            break;
        SE3 Tcw = pFrame->Tcw_.inverse();

        // show map and camera pose
        cv::Affine3d M(
            cv::Affine3d::Mat3(
                Tcw.rotation_matrix()(0, 0), Tcw.rotation_matrix()(0, 1), Tcw.rotation_matrix()(0, 2),
                Tcw.rotation_matrix()(1, 0), Tcw.rotation_matrix()(1, 1), Tcw.rotation_matrix()(1, 2),
                Tcw.rotation_matrix()(2, 0), Tcw.rotation_matrix()(2, 1), Tcw.rotation_matrix()(2, 2)),
            cv::Affine3d::Vec3(
                Tcw.translation()(0, 0), Tcw.translation()(1, 0), Tcw.translation()(2, 0)));
        cv::imshow("image", color);
        cv::waitKey(1);
        vis.setWidgetPose("camera", M);
        vis.spinOnce(1, false);
    }
    return 0;
}