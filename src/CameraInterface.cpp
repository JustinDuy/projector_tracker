
#include "projector_tracker/CameraInterface.h"
#include <iostream>

CameraInterface::CameraInterface(int device_number)
{    
    device_number = 1;
    capture = cv::VideoCapture(device_number);
    if(!capture.isOpened() )
    {
      std::cerr << "Error: camera device is not opened!" << std::endl;
    }
    else
      std::cout << "camera device is opened" << std::endl;
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    calibration.width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    calibration.height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    //std::cout << calibration.width << calibration.height << std::endl;
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
    int width, height;
    fs["imageSize_width"] >> width ;
    fs["imageSize_height"] >> height;
    calibration.height = height;
    calibration.width = width;
    return true;
}

CameraInterfaceBase::Calibration CameraInterface::getCalibration()
{
    return calibration;
}


cv::Mat CameraInterface::grabFrame() {
    // TODO: rectify image if intrinsics are loaded
    cv::Mat frame = cv::Mat::zeros(calibration.height,calibration.width,CV_8UC3);
    std::cout << "capturing frame " << std::endl;
    capture >> frame;
    if(frame.empty()) {
        std::cout << "frame is empty" << std::endl;
    }
    else
    {
        cv::cvtColor( frame, frame, CV_RGB2GRAY );
        std::cout << "captured" << std::endl;
    }
    return frame;
}

