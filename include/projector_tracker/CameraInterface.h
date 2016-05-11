
#ifndef CAMERAINTERFACE_H
#define CAMERAINTERFACE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

/**
 * @brief Allows to capture images from a video camera
 * 
 */
class CameraInterface
{
public:
    /**
     * @brief Constructs a CameraInterface, identifying the target screen.
     * 
     */
    CameraInterface(std::string calibration_filepath , std::string tag, int device_number = 0);
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~CameraInterface();
    
public:
    bool loadIntrinsics(std::string file, std::string tag);
    cv::Mat getIntrinsics();
    cv::Mat grabFrame();
    
protected:
    cv::Mat intrinsics;  /// intrinsic camera calibration parameters
    cv::VideoCapture capture; // Camera Handle
    unsigned int width;
    unsigned int height;
};

#endif // CAMERAINTERFACE_H
