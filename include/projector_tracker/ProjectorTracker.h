
#ifndef PROJECTORTRACKER_H
#define PROJECTORTRACKER_H

#include <opencv2/core/core.hpp>
#include <vector>

/**
 * @brief Camera-Projector relative position tracker
 * 
 */
class ProjectorTracker {
public:
    /**
    * @brief Creates a ProjectorTracker for a camera and a projector with the given intrinsic parameters.
    * 
    * @param camera_intrinsics Intrinsic calibration of the RGB camera used to track the projector position.
    * @param projector_intrinsics Intrinsic calibration of the projector that should be tracked.
    */
    ProjectorTracker(const cv::Mat& camera_intrinsics, const cv::Mat& projector_intrinsics);
    
    /**
    * @brief Creates a Gray code pattern to be projected by the projector, according to its intrinsics.
    * 
    * @return cv::Mat
    */
    std::vector<cv::Mat> getPattern(int width, int height);
    
    /**
    * @brief Given the current frame grabbed by the camera, compute the relative position of the projector.
    * 
    * @param camera_image Current frame as grabbed by the RGB camera.
    * @return cv::Mat
    */
    cv::Mat computeRelativePosition(const std::vector<cv::Mat>& camera_image);
    
protected:
    cv::Mat camera_intrinsics;  /// intrinsic RGB camera calibration parameters
    cv::Mat projector_intrinsics;  /// intrinsic projector calibration parameters

private:
    cv::Mat pattern;  /// cached pattern
};

#endif // PROJECTORTRACKER_H
