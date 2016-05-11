
#ifndef CAMERAINTERFACE_H
#define CAMERAINTERFACE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

class CameraInterfaceBase {
public:
    cv::Mat grabFrame() = 0;
    cv::Mat getIntrinsics() = 0;
    unsigned int getWidth() = 0;
    unsigned int getHeight() = 0;
};

/**
 * @brief Allows to capture images from a video camera
 * 
 */
class CameraInterface : public CameraInterfaceBase
{
public:
    /**
     * @brief Constructs a CameraInterface, identifying the target screen.
     * 
     */
    CameraInterface(int device_number = 0);
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~CameraInterface();
    
public:
    bool loadIntrinsics(std::string file, std::string tag);
    cv::Mat getIntrinsics();
    unsigned int getWidth() { return width; }
    unsigned int getHeight() { return height; }
    cv::Mat grabFrame();
    
protected:
    cv::Mat intrinsics;  /// intrinsic camera calibration parameters
    cv::VideoCapture capture; // Camera Handle
    unsigned int width;
    unsigned int height;
};

#endif // CAMERAINTERFACE_H
