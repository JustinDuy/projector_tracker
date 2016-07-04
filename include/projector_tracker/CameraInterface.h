
#ifndef CAMERAINTERFACE_H
#define CAMERAINTERFACE_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

class CameraInterfaceBase {
public:
    struct Calibration {
        unsigned int width;
        unsigned int height;
        cv::Mat intrinsics;
    };

    virtual cv::Mat grabFrame() = 0;
    virtual Calibration getCalibration() = 0;
};

/**
 * @brief Allows to capture images from a video camera
 * 
 */
class CameraInterface : public CameraInterfaceBase
{
public:
    using CameraInterfaceBase::Calibration;
    
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
    bool loadIntrinsics(std::string file, std::string tag_K, std::string tag_W, std::string tag_H);
    Calibration getCalibration();
    cv::Mat grabFrame();
    
protected:
    Calibration calibration;  /// intrinsic camera calibration parameters
    cv::VideoCapture capture; // Camera Handle
};

#endif // CAMERAINTERFACE_H
