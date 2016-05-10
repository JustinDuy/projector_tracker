#include "projector_tracker/CameraInterface.h"
#include <iostream>

CameraInterface::CameraInterface()
{

}
bool CameraInterface::loadIntrinsic(std::string matrix_file )
{
    cv::FileStorage fs( matrix_file, cv::FileStorage::READ );
    if( !fs.isOpened() )
    {
      std::cout << "Failed to open Camera Calibration Data File." << std::endl;
      return false;
    }
    // Loading calibration parameters
    fs["camera_matrix"] >> intrinsics;
    return true;
}
cv::Mat CameraInterface::get(){
    return cv::Mat(intrinsics);
}
CameraInterface::~CameraInterface()
{

}


