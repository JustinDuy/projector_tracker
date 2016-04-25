
#ifndef PROJECTORINTERFACE_H
#define PROJECTORINTERFACE_H

#include <opencv2/core/core.hpp>

/**
 * @brief Allows to project images on a projector at full screen.
 * 
 */
class ProjectorInterface
{
public:
    /**
     * @brief Constructs a ProjectorInterface, identifying the target screen.
     * 
     */
    ProjectorInterface(const cv::Mat& projector_intrinsics);
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~ProjectorInterface();
    
protected:
    cv::Mat projector_intrinsics;  /// intrinsic projector calibration parameters
    
    // TODO: full-screen window?
};

#endif // PROJECTORINTERFACE_H
