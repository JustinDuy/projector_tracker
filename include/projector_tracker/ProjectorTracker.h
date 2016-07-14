
#ifndef PROJECTORTRACKER_H
#define PROJECTORTRACKER_H

#include <projector_tracker/CameraProjectorInterface.h>
#include <opencv2/core.hpp>

const unsigned int DEFAULT_BLACK_THRESHOLD = 40;  // 3D_underworld default value 40
const unsigned int DEFAULT_WHITE_THRESHOLD = 5;   // 3D_underworld default value  5


/**
 * @brief Camera-Projector relative position tracker
 * 
 */
class ProjectorTracker {
public:
    using CameraProjectorImagePair = CameraProjectorInterface::CameraProjectorImagePair;
    
    /**
    * @brief Creates a ProjectorTracker using a given CameraProjectorInterface
    * 
    * @param cp_interface Camera-Projector interface used to project and acquire images
    */
    ProjectorTracker(std::shared_ptr<CameraProjectorInterface> cp_interface);
    
    /**
    * @brief Creates a Gray code pattern to be projected by the projector, according to its intrinsics.
    * 
    * @return cv::Mat
    */
    std::vector<cv::Mat> getPatternImages(int width, int height, bool useAruco);

    /**
    * @brief backproject the 2D points on camera image to 3D object points giving Extrinsic and Intrinsic Matrices
    *
    * @param const cv::Mat& K,cv::Mat& boardRot64,const cv::Mat& boardTrans64, const vector<cv::Point2f>& imgPt
    * @return bool
    */
    bool backProject(const cv::Mat& K, const cv::Mat& boardRot64,
                                        const cv::Mat& boardTrans64,
                                        const std::vector<cv::Point2f>& imgPt,
                                        std::vector<cv::Point3f>& worldPt);
    /**
    * @brief Given the current frame grabbed by the camera, compute the relative position of the projector.
    * 
    * @param camera_image Current frame as grabbed by the RGB camera.
    * @return cv::Mat
    */
    cv::Mat computeRelativePosition(const std::vector<CameraProjectorImagePair>& camera_image, bool useAruco);

protected:
    std::shared_ptr<CameraProjectorInterface> cp_interface;  // Gives synchronized access to camera and projector

private:
    cv::Mat pattern;  /// cached pattern
    void saveExtrinsics(cv::Mat R, cv::Mat t, std::string filename) ;
};

#endif // PROJECTORTRACKER_H
