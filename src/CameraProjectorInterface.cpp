
#include <projector_tracker/CameraProjectorInterface.h>

CameraProjectorInterface::CameraProjectorInterface(const cv::Mat& camera_intrinsics, const cv::Mat& projector_intrinsics)
: camera_intrinsics(camera_intrinsics)
, projector_intrinsics(projector_intrinsics)
{ }

CameraProjectorInterface::~CameraProjectorInterface()
{
    // TODO: release camera and projector if necessary
}


cv::Mat CameraProjectorInterface::projectAndAcquire(const cv::Mat& target_image)
{
    // TODO: send the target_image to the projector, wait for it to project it, grab through camera
}

std::vector<cv::Mat> CameraProjectorInterface::projectAndAcquire(const std::vector<cv::Mat>& target_images)
{
    std::vector<cv::Mat> ret;
    for (auto& target_image: target_images)
        ret.push_back(projectAndAcquire(target_image));
    return ret;
}
