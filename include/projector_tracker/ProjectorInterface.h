
#ifndef PROJECTORINTERFACE_H
#define PROJECTORINTERFACE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QLabel>
#include <string>

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
    ProjectorInterface();
    /**
     * @brief Releases the acquired resources.
     * 
     */
    ~ProjectorInterface();
    
    void projectFullscreen(const cv::Mat& target_image, int screen_number = 1);

public:
    bool loadIntrinsic(std::string file, std::string tag);
    cv::Mat getIntrinsics();
protected:
    cv::Mat intrinsics;  /// intrinsic projector calibration parameters
    
    QLabel image_label;  // used to display an image
};

#endif // PROJECTORINTERFACE_H
