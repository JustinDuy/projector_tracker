
#ifndef CAMERAPROJECTORINTERFACE_H
#define CAMERAPROJECTORINTERFACE_H

#include <opencv2/core/core.hpp>

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
    CameraProjectorInterface(const cv::Mat& camera_intrinsics, const cv::Mat& projector_intrinsics);
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~CameraProjectorInterface();
    
    
    cv::Mat projectAndAcquire(const cv::Mat& target_image);
    std::vector<cv::Mat> projectAndAcquire(const std::vector<cv::Mat>& target_images);
    
protected:
    cv::Mat camera_intrinsics;  /// intrinsic RGB camera calibration parameters
    cv::Mat projector_intrinsics;  /// intrinsic projector calibration parameters
};

#endif // PROJECTORINTERFACE_H
