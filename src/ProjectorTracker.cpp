
#include <projector_tracker/ProjectorTracker.h>
#include <opencv2/structured_light.hpp>

ProjectorTracker::ProjectorTracker(const cv::Mat& camera_intrinsics, const cv::Mat& projector_intrinsics)
: camera_intrinsics(camera_intrinsics)
, projector_intrinsics(projector_intrinsics)
{ }


std::vector<cv::Mat> ProjectorTracker::getPattern()
{

}


cv::Mat ProjectorTracker::computeRelativePosition(const std::vector<cv::Mat>& camera_image)
{

}
