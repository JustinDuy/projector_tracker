
#ifndef CAMERAINTERFACE_H
#define CAMERAINTERFACE_H
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/**
 * @brief Allows to project images on a projector at full screen.
 * 
 */
class CameraInterface
{
public:
    /**
     * @brief Constructs a ProjectorInterface, identifying the target screen.
     * 
     */
    CameraInterface();
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~CameraInterface();
public:
    bool loadIntrinsic(std::string file);
    cv::Mat get();
protected:
    cv::Mat intrinsics;  /// intrinsic projector calibration parameters
    
    // TODO: full-screen window?
};

#endif // CAMERAINTERFACE_H
