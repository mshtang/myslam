#pragma once

#include "myslam/common_include.h"

namespace myslam
{
class Camera
{
public:
  typedef std::shared_ptr<Camera> Ptr;

  // camera intrinsics
  float fx_, fy_, cx_, cy_, depth_scale_;

  Camera();
  Camera(float fx, float fy, float cx, float cy, float depth_scale = 0) : fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {}

  Vector3d world2camera(const Vector3d &pw, const SE3 &Tcw);
  Vector2d camera2pixel(const Vector3d &pc);
  Vector3d pixel2camera(const Vector2d &pp, double depth = 1);
  Vector3d camera2world(const Vector3d &pc, const SE3 &Tcw);
  Vector2d world2pixel(const Vector3d &pw, const SE3 &Tcw);
  Vector3d pixel2world(const Vector2d &pp, const SE3 &Tcw, double depth = 1);
};
} // namespace myslam
