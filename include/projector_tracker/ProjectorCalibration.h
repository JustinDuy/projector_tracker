/*
 * ProjectorCalibration.h
 *
 *  Created on: Aug 24, 2016
 *      Author: justin
 */

#ifndef PROJECTORCALIBRATOR_H_
#define PROJECTORCALIBRATOR_H_
#include <projector_tracker/CameraProjectorInterface.h>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;
enum CalibrationPattern {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};
class ProjectorCalibration {
public:
	void setStaticCandidateImagePoints();
	void loadCamIntrinsic(string camcalib, string tag_k, string tag_d);
	bool loadSetting(string configFile, string tag_w, string tag_h, string tag_pattern_type, string tag_pattern_w,
			string tag_pattern_h,string tag_square_sz, string tag_x, string tag_y, string tag_checker_w, string tag_checker_h,
			string tag_checker_square, string tag_max_reprojection_error,
			string tag_num_clean, string tag_num_final,
			string camera_matrix, string tag_k, string tag_d);
	void computeCandidateBoardPose(const vector<cv::Point2f> & imgPts,const vector<cv::Point3f> & objPts, cv::Mat& boardRot, cv::Mat& boardTrans);
	bool backProject(const cv::Mat& boardRot64, const cv::Mat& boardTrans64,
					 const vector<cv::Point2f>& imgPt,
					 vector<cv::Point3f>& worldPt);

	void drawCheckerBoard(const Mat& img, vector<Point2f> pointBuf);
	void drawCircleGrid(const cv::Mat& img, vector<cv::Point2f> checkerpointBuf, vector<cv::Point2f> circlepointBuf);
	bool add(const Mat& img, const Mat& processedImg,  vector<Point2f>& circlesImgPts) ;
	bool calibrate();
	//bool findBoard(cv::Mat img, std::vector<cv::Point2f> &pointBuf);
	void save(std::string filename, bool absolute = false) const;
	int size() const;
	bool clean();
	float getReprojectionError() const;
	float getReprojectionError(int i) const;
	void updateReprojectionError();
	//void updateImagePoints() ;
	bool isReady();
	void updateUndistortion();
	void setPatternPosition(float px, float py) ;
	static std::vector<cv::Point2f> createImagePoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType);
	static std::vector<Point3f> createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType);
	float maxReprojectionError;
	int numBoardsFinalCamera;
	int numBoardsBeforeCleaning;
	void getPattern(Mat& out);
        void updateImagePoints();
private:
	vector<cv::Point2f> candidateImagePoints;
     	std::vector<std::vector<cv::Point2f> > cam_imagePoints;
  
	std::vector<std::vector<cv::Point2f> > imagePoints;
        std::vector<std::vector<cv::Point3f> > objectPoints;
protected:

	int projectorWidth;
	int projectorHeight;
	bool ready;
        
	cv::Size addedImageSize;
	cv::Size ImagerSize;

	//circle grid parameters:
	CalibrationPattern patternType;
	cv::Size patternSize;
	float squareSize;
	cv::Point2f patternPosition ;//start drawing pattern at (corner_x, corner_y) on projector screen

	//checkboard parameters:
	cv::Size checkerBoardSize;
	float checkerSquareSize;
	cv::Mat grayMat;

	std::vector<cv::Mat> boardRotations, boardTranslations;
	float reprojectionError;
	std::vector<float> perViewErrors;
        
	//camera intrinsic
        cv::Mat cam_distortedIntrinsics;
	cv::Mat cam_distortion_coeffs;

        
	//projector intrinsic
	cv::Mat distortedIntrinsics;
	cv::Mat undistortedIntrinsics;
        cv::Mat distCoeffs;

        //extrinsic cam to projector
        cv::Mat rotCamToProj;
        cv::Mat transCamToProj;
        cv::Mat fundamentalMatrix, essentialMatrix;
};

#endif /* PROJECTORCALIBRATOR_H_ */
