
#ifndef PROJECTORTRACKER_H
#define PROJECTORTRACKER_H

#include <projector_tracker/CameraProjectorInterface.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco/charuco.hpp>
const unsigned int DEFAULT_BLACK_THRESHOLD = 40;  // 3D_underworld default value 40
const unsigned int DEFAULT_WHITE_THRESHOLD = 5;   // 3D_underworld default value  5
using namespace std;
using namespace cv;
enum CalibrationPattern {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID, ARUCO, GRAYCODE};
/**
 * @brief Camera-Projector relative position tracker
 * 
 */
class ProjectorTracker {
public:
    using CameraProjectorImagePair = CameraProjectorInterface::CameraProjectorImagePair;
    
    /**
    * @brief Creates a ProjectorTracker 
    */
    ProjectorTracker();
    /**
    * @brief Load Setting
    *
    * @param std::string matrix_file, std::string tag,...
    */
    bool loadSetting(std::string matrix_file , std::string tag_patternType, std::string tag_knownObj, std::string tag_W, std::string tag_H, std::string tag_squareSize, std::string tag_x, std::string tag_y,
                                   std::string tag_CheckerPatternW, std::string tag_CheckerPatternH, std::string tag_CheckerSquare,
                                   std::string tag_in_reproj_err, std::string tag_ex_reproj_err, std::string tag_num_before_clean, std::string tag_num_before_calib,
                                   std::string cam_intrinsic, std::string tag_cam_k, std::string tag_cam_d,  std::string tag_cam_w, std::string tag_cam_h, 
                                   std::string pro_intrinsic, std::string tag_pro_k, std::string tag_pro_d ,std::string tag_pro_w, std::string tag_pro_h
                                  );
    void loadIntrinsic(string camcalib, string tag_k, string tag_d,  std::string tag_w, std::string tag_h, bool forProjector);
    void saveExtrinsic(string filename) const;
    void saveProjectorIntrinsic(string filename) const ;
    
    /**
    * @brief get rotation from camera to projector if the calibration is ready
    * @output rot
    * @return false if calibration not ready
    */
    bool getCamToProjRotation(Mat& rot);
    
    /**
    * @brief get translation from camera to projector if the calibration is ready
    * @output translation
    * @return false if calibration not ready
    */
    bool getCamToProjTranslation(Mat& trans);
    
        
    /**
    * @brief do the calibration according to mode defined in setting.yml
    * @input patternImg (projector image) and projected image that is captured by the camera
    * @return false if calibration not ready
    */
    bool run(const Mat& patternImg, const Mat& projectedImg);
    
    /**
    * @brief Creates a Gray code pattern/Aruco board to be projected by the projector
    * 
    * @return std::vector<cv::Mat>
    */
    std::vector<cv::Mat> getPatternImages();


protected:
    bool interpolate3D(const cv::Mat& img, const vector<cv::Point2f> checkercorners, const vector<Point2f> markers, vector<Point3f>& marker_objectPts);
    Mat prevMat;
    bool updateCamDiff(cv::Mat camMat);
    int size() const;
    void clear();
    bool unknown3DObj_calib(const Mat& patternImg, const Mat& captured);
    bool addProjected2D(const Mat& patternImg, const Mat& projectedImg);
    void processImageForCircleDetection(const Mat& img, Mat& processedImg);
    void drawCircleGrid(const cv::Mat& img, vector<cv::Point2f> circlePointBuf);
    bool known3DObj_calib(const Mat& patternImg, const Mat& captured);
    bool addProjected(const cv::Mat& patternImg, const cv::Mat& projectedImg);
    bool addProjected_aruco(const cv::Mat& patternImg, const cv::Mat& projectedImg);
    bool addProjected_circlegrid(const cv::Mat& patternImg, const cv::Mat& projectedImg);
    bool stereoCalibrate();
    bool calibrateProjector();
    float getReprojectionError() const;
    float getReprojectionError(int i) const;
    int cleanStereo();
    
    
    float maxExtrinsicReprojectionError;
    float maxIntrinsicReprojectionError;
    int numBoardsFinalCamera;
    int numBoardsBeforeCleaning;
    
    static std::vector<Point3f> createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType);
    void updateReprojectionError();
    
    void drawCheckerBoard(const cv::Mat& img, std::vector<cv::Point2f> checkerCorners, cv::Mat& out);
    void drawAruco_board(const cv::Mat& image, std::vector<vector<cv::Point2f> >  markerCorners, std::vector<int> markerIds, std::vector<cv::Point2f> interpolated_corners,std::vector<int> interpolated_ids, cv::Mat& out);
    void drawAruco(const cv::Mat& image, std::vector<vector<cv::Point2f> >  markerCorners, std::vector<int> markerIds, cv::Mat& out);
    bool findAruco_board(const cv::Mat& image, std::vector<vector<cv::Point2f> > & markerCorners, std::vector<int>& markerIds,  std::vector<cv::Point2f> & interpolated_charucoCorners,  std::vector<int>& interpolated_charucoIds);
    bool findAruco(const cv::Mat& image, vector<cv::Point2f>& markerPts, vector<vector<cv::Point2f> > & markerCorners, std::vector<int>& markerIds);
    /**
    * @brief find the 2D points of calibration board on camera image
    * @param const Mat& img, bool refine
    * @return bool
    */
    //bool findBoard(const cv::Mat& img, std::vector<cv::Point2f>& pointBuf, bool refine);
    /**
    * @brief detect charuco board on projector and camera image to establish correspondence
    * @param const cv::Mat& projected, const cv::Mat& acquired
    * @return int
    */
    //void findCorrespondence_Aruco(const std::vector<CameraProjectorImagePair>& cp_images, std::vector<std::vector<cv::Point2f> >& camPixels, std::vector<std::vector<cv::Point2f> >& projPixels);
    //int findCorrespondence_GrayCode(const std::vector<CameraProjectorImagePair>& cp_images, std::vector<cv::Point2f>& camPixels, std::vector<cv::Point2f>& projPixels);
    /**
    * @brief compute Board rotation and translation according to camera plane
    * @param const vector<cv::Point2f> & img3DPts,
    *        const vector<cv::Point2f> & imgPts,
             cv::Mat& K, cv::Mat distCoeffs
    * @return void
    */
    void computeCandidateBoardPose(const std::vector<cv::Point2f> & imgPts,const std::vector<cv::Point3f> & img3DPts, 
                                                     const cv::Mat& K, const cv::Mat& distCoeffs, cv::Mat& boardRot, cv::Mat& boardTrans);
    /**
    * @brief backproject the 2D points on camera image to 3D object points giving Extrinsic and Intrinsic Matrices
    *
    * @param const cv::Mat& K,cv::Mat& boardRot64,const cv::Mat& boardTrans64, const vector<cv::Point2f>& imgPt
    * @return bool
    */
    bool backProject(const cv::Mat& boardRot64,
                                        const cv::Mat& boardTrans64,
                                        const std::vector<cv::Point2f>& imgPt,
                                        std::vector<cv::Point3f>& worldPt);

    std::vector<std::vector<cv::Point3f> > objectPoints;
    std::vector<std::vector<cv::Point2f> > cam_imgPoints;
    std::vector<std::vector<cv::Point2f> > pro_imgPoints;
    cv::Size addedImageSize;
    cv::Size ImagerSize;
    std::vector<cv::Mat> boardRotations, boardTranslations;
    float reprojectionError;
    std::vector<float> perViewErrors;
    cv::Mat projectorMatrix     ;
    cv::Mat projectorDistCoeffs ;
    cv::Mat cameraMatrix        ;
    cv::Mat cam_undistorted_intrinsics ;
    cv::Mat cameraDistCoeffs    ;
    
    //extrinsic cam to projector
    cv::Mat rotCamToProj;
    cv::Mat transCamToProj;
    cv::Mat fundamentalMatrix, essentialMatrix;
    //camera pose Rot&Trans
    std::vector<cv::Mat> cam_board_rot;
    std::vector<cv::Mat> cam_board_trans;
    
    std::vector<cv::Point2f> getPatternPoints();
private:
    cv::Mat pattern;  /// cached pattern

    bool known3DObj;
    CalibrationPattern patternType;

    aruco::Dictionary dictionary ;
    aruco::CharucoBoard  board ;
    
    cv::Size checkerBoardSize;
    double checkerSquareSize;
    
    cv::Size patternSize ;
    double squareSize ;
    cv::Point2f patternPosition ;//start drawing pattern at (corner_x, corner_y) on projector screen;
    
    bool intrinsic_ready;
    bool extrinsic_ready;
};

#endif // PROJECTORTRACKER_H
