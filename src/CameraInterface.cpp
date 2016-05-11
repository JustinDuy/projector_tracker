
#include "projector_tracker/CameraInterface.h"
#include <iostream>

CameraInterface::CameraInterface(std::string matrix_file, std::string tag, int device_number)
{
    
    if( !loadIntrinsics(matrix_file, tag) )
    {
      std::cerr << "Error: could not load camera intrinsic calibration from file " << matrix_file << std::endl;
    }
    
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

cv::Mat CameraInterface::grabFrame() {
    cv::Mat frame;
    capture >> frame;
    return frame;
}

cv::Mat CameraInterface::getIntrinsics()
{
    return cv::Mat(intrinsics);
}

CameraInterface::~CameraInterface()
{

}


