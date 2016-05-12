
#include "projector_tracker/CameraInterface.h"
#include <iostream>

CameraInterface::CameraInterface(int device_number)
{    
    capture = cv::VideoCapture(device_number);
    if( !capture.isOpened() )
    {
      std::cerr << "Error: camera device is not opened!" << std::endl;
    }
    calibration.width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    calibration.height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

CameraInterface::~CameraInterface()
{

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
    fs[tag] >> calibration.intrinsics;
    return true;
}

CameraInterfaceBase::Calibration CameraInterface::getCalibration()
{
    return calibration;
}


cv::Mat CameraInterface::grabFrame() {
    // TODO: rectify image if intrinsics are loaded
    cv::Mat frame;
    capture >> frame;
    return frame;
}

