
#ifndef PROJECTORINTERFACE_H
#define PROJECTORINTERFACE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QLabel>
#include <string>

class ProjectorInterfaceBase {
public:
    struct Calibration {
        unsigned int width;
        unsigned int height;
        cv::Mat intrinsics;
    };
    
    virtual void projectFullscreen(const cv::Mat& target_image) = 0;
    virtual Calibration getCalibration() = 0;
};

/**
 * @brief Allows to project images on a projector at full screen.
 * 
 */
class ProjectorInterface : public ProjectorInterfaceBase
{
public:
    using ProjectorInterfaceBase::Calibration;
    
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
    
    void projectFullscreen(const cv::Mat& target_image) { projectFullscreenOnScreen(target_image, 1); }
    void projectFullscreenOnScreen(const cv::Mat& target_image, int screen_number = 1);

public:
    bool loadIntrinsics(std::string file, std::string tag);
    Calibration getCalibration();
    
protected:
    Calibration calibration;  /// intrinsic projector calibration parameters
    
    QLabel image_label;  // used to display an image
};

#endif // PROJECTORINTERFACE_H
