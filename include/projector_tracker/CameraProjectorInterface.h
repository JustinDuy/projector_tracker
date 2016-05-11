
#ifndef CAMERAPROJECTORINTERFACE_H
#define CAMERAPROJECTORINTERFACE_H

#include <projector_tracker/CameraInterface.h>
#include <projector_tracker/ProjectorInterface.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * @brief Allows synchronized projection of a pattern and acquisition through a camera.
 * 
 */
class CameraProjectorInterface
{
public:
    /**
     * @brief Constructs a CameraProjectorInterface, identifying the target screen and cameras.
     * 
     */
    CameraProjectorInterface(const cv::Mat& camera_intrinsics, size_t w, size_t h, const cv::Mat& projector_intrinsics, int capture_device);
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~CameraProjectorInterface();
    
    
    cv::Mat projectAndAcquire(const cv::Mat& target_image, int delay, int seq);
    std::vector<cv::Mat> projectAndAcquire(const std::vector<cv::Mat>& target_images);
    
protected:
    
};

#endif // CAMERAPROJECTORINTERFACE_H
