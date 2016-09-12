
#include <projector_tracker/CameraProjectorInterface.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
using namespace std;
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
    //std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    do 
    {
        cout << '\n' << "Press a key to capture...";
    } 
    while (cin.get() != '\n');
    ret.acquired = camera->grabFrame();
    return ret;
}

std::vector<CameraProjectorInterface::CameraProjectorImagePair> CameraProjectorInterface::projectAndAcquire(const std::vector<cv::Mat>& target_images)
{
    std::vector<CameraProjectorImagePair> ret;
    int count = 0;
    char file[500];
    for (auto& target_image: target_images){
        CameraProjectorImagePair pair = projectAndAcquire(target_image);
        sprintf(file,"captured_%d.jpeg",++count);
        ret.push_back(pair);
        cv::imwrite(file, pair.acquired);
    }
    return ret;
}
