
#include "projector_tracker/CameraInterface.h"
#include <iostream>

CameraInterface::CameraInterface(int device_number)
{    
    //device_number = 1;
    capture = cv::VideoCapture(device_number);
    if(!capture.isOpened() )
    {
      std::cerr << "Error: camera device is not opened!" << std::endl;
    }
    //only for cam calibration
    capture.set(CV_CAP_PROP_FPS,10);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
     
    std::cout << "init camera done" << std::endl;
}

CameraInterface::~CameraInterface()
{

}

bool CameraInterface::loadIntrinsics(std::string matrix_file , std::string tag_K, std::string tag_D, std::string tag_W, std::string tag_H)
{
    cv::FileStorage fs( matrix_file, cv::FileStorage::READ );
    if( !fs.isOpened() )
    {
      std::cout << "Failed to open Camera Calibration Data File." << std::endl;
      return false;
    }
    // Loading calibration parameters
    fs[tag_K] >> calibration.intrinsics;
    fs[tag_D] >> calibration.distortion_coeffs;
    int width, height;
    fs[tag_W] >> width ;
    fs[tag_H] >> height;
    calibration.height = height;
    calibration.width = width;
    capture.set(CV_CAP_PROP_FPS,10);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    return true;
}

CameraInterfaceBase::Calibration CameraInterface::getCalibration()
{
    return calibration;
}


cv::Mat CameraInterface::grabFrame() {
    // TODO: rectify image if intrinsics are loaded
    cv::Mat frame = cv::Mat::zeros(calibration.height, calibration.width, CV_8UC3);
    capture >> frame;
    //cv::cvtColor( frame, frame, CV_RGB2GRAY );
    return frame;
}

