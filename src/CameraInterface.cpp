
#include "projector_tracker/CameraInterface.h"
#include <iostream>

CameraInterface::CameraInterface(int device_number)
{    
    capture = cv::VideoCapture(device_number);
    if( !capture.isOpened() )
    {
      std::cerr << "Error: camera device is not opened!" << std::endl;
    }
    width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

bool CameraInterface::loadIntrinsics(std::string matrix_file , std::string tag)
{
    cv::FileStorage fs( matrix_file, cv::FileStorage::READ );
    if( !fs.isOpened() )
    {
      std::cout << "Failed to open Camera Calibration Data File." << std::endl;
      return false;
    }
    // Loading calibration parameters
    fs[tag] >> intrinsics;
    return true;
}

cv::Mat CameraInterface::getIntrinsics()
{
    return cv::Mat(intrinsics);
}

cv::Mat CameraInterface::grabFrame() {
    cv::Mat frame;
    capture >> frame;
    return frame;
}

CameraInterface::~CameraInterface()
{

}


