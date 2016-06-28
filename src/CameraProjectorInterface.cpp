
#include <projector_tracker/CameraProjectorInterface.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

CameraProjectorInterface::CameraProjectorInterface(std::shared_ptr<CameraInterfaceBase> camera_interface, std::shared_ptr<ProjectorInterfaceBase> projector_interface, int delay_ms)
: camera(camera_interface)
, projector(projector_interface)
, delay_ms(delay_ms)
{
    
}


CameraProjectorInterface::~CameraProjectorInterface()
{ 
    
}

CameraProjectorInterface::CameraProjectorImagePair CameraProjectorInterface::projectAndAcquire(const cv::Mat& target_image)
{
    CameraProjectorImagePair ret;
    ret.projected = target_image;
    projector->projectFullscreen(target_image);
    std::cout << "delay ms " << delay_ms << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    
    ret.acquired = camera->grabFrame();
    
    return ret;
}

std::vector<CameraProjectorInterface::CameraProjectorImagePair> CameraProjectorInterface::projectAndAcquire(const std::vector<cv::Mat>& target_images)
{
    std::vector<CameraProjectorImagePair> ret;
    
    for (auto& target_image: target_images)
        ret.push_back(projectAndAcquire(target_image));
    
    return ret;
}
