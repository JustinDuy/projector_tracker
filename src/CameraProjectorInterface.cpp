
#include <projector_tracker/CameraProjectorInterface.h>
#include <string>
#include <iostream>

CameraProjectorInterface::CameraProjectorInterface(int webcam_device)
{
    
}

CameraProjectorInterface::~CameraProjectorInterface()
{ }


cv::Mat CameraProjectorInterface::projectAndAcquire(const cv::Mat& target_image, int delay_ms, int seq)
{
    // TODO: send the target_image to the projector, wait for it to project it, grab through camera
}

std::vector<cv::Mat> CameraProjectorInterface::projectAndAcquire(const std::vector<cv::Mat>& target_images)
{
    // Setting pattern window on second monitor (the projector's one)
    cv::namedWindow( "Pattern Window", cv::WINDOW_NORMAL );
    cv::resizeWindow( "Pattern Window", camera_width*2, camera_height*2 );
    cv::moveWindow( "Pattern Window", camera_width + 1000, 50 );
    std::vector<cv::Mat> ret;
    int i =0;
    for (auto& target_image: target_images){
        ret.push_back(projectAndAcquire(target_image, i == 0 ? 500 : 250 , i+1));
        i++;
    }
    return ret;
}
