#include "myslam/camera.h"

namespace myslam
{
Camera::Camera() {}

Vector3d Camera ::world2camera(const Vector3d &pw, const SE3 &Tcw)
{
    return Tcw * pw;
}

Vector2d Camera::camera2pixel(const Vector3d &pc)
{
    return Vector2d(
        pc.x() * fx_ / pc.z() + cx_,
        pc.y() * fy_ / pc.z() + cy_);
}

Vector3d Camera::pixel2camera(const Vector2d &pp, double depth)
{
    return Vector3d(
        (pp.x() - cx_) * depth / fx_,
        (pp.y() - cy_) * depth / fy_,
        depth);
}

Vector3d Camera::camera2world(const Vector3d &pc, const SE3 &Tcw)
{
    return Tcw.inverse() * pc;
}

Vector2d Camera::world2pixel(const Vector3d &pw, const SE3 &Tcw)
{
    return (camera2pixel(world2camera(pw, Tcw)));
}

Vector3d Camera::pixel2world(const Vector2d &pp, const SE3 &Tcw, double depth)
{
    return camera2world(pixel2camera(pp, depth), Tcw);
}
} // namespace myslam