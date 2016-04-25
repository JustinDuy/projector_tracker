
#include "projector_tracker/ProjectorInterface.h"

ProjectorInterface::ProjectorInterface(const cv::Mat& projector_intrinsics)
: projector_intrinsics(projector_intrinsics)
{

}

ProjectorInterface::~ProjectorInterface()
{

}
