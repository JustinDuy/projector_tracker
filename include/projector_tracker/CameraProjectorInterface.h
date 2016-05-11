
#ifndef CAMERAPROJECTORINTERFACE_H
#define CAMERAPROJECTORINTERFACE_H

#include <projector_tracker/CameraInterface.h>
#include <projector_tracker/ProjectorInterface.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <memory>

const int DEFAULT_PROJ_ACQ_DELAY_MS = 300;

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
    CameraProjectorInterface(std::shared_ptr<CameraInterfaceBase> camera_interface, std::shared_ptr<ProjectorInterfaceBase> projector_interface, int delay_ms = DEFAULT_PROJ_ACQ_DELAY_MS);
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~CameraProjectorInterface();
    
    
    cv::Mat projectAndAcquire(const cv::Mat& target_image);
    std::vector<cv::Mat> projectAndAcquire(const std::vector<cv::Mat>& target_images);
    
protected:
    std::shared_ptr<CameraInterfaceBase> camera;
    std::shared_ptr<ProjectorInterfaceBase> projector;
    int delay_ms;
};

#endif // CAMERAPROJECTORINTERFACE_H
