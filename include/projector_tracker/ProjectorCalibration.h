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
	bool loadSetting(string configFile, string tag_pattern_type, string tag_pattern_w,
			string tag_pattern_h,string tag_square_sz, string tag_checker_w, string tag_checker_h,
			string tag_checker_square, string tag_max_reprojection_error,
			string tag_num_clean, string tag_num_final,
			string camera_matrix, string tag_k, string tag_d);
	void computeCandidateBoardPose(const vector<cv::Point2f> & imgPts,const vector<cv::Point3f> & objPts, cv::Mat& boardRot, cv::Mat& boardTrans);
	bool backProject(const cv::Mat& boardRot64, const cv::Mat& boardTrans64,
					 const vector<cv::Point2f>& imgPt,
					 vector<cv::Point3f>& worldPt);

	vector<cv::Point3f> getCandidateObjectPoints() { return candidateObjectPts; }
	void drawCircleGrid(cv::Mat img, vector<cv::Point2f> pointBuf);
	bool add(Mat img, Mat processedImg,  vector<Point2f>& circlesImgPts) ;
	bool calibrate();
	bool findBoard(cv::Mat img, std::vector<cv::Point2f> &pointBuf);
	void save(std::string filename, bool absolute = false) const;
	int size() const;
	bool clean();
	float getReprojectionError() const;
	float getReprojectionError(int i) const;
	void updateReprojectionError();
	void updateImagePoints() ;
	bool isReady();
	void updateUndistortion();
	void setPatternPosition(float px, float py) ;
	static std::vector<cv::Point2f> createImagePoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType);
	static std::vector<Point3f> createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType);
	float maxReprojectionError;
	int numBoardsFinalCamera;
	int numBoardsBeforeCleaning;
	void getPattern(Mat& out);
private:
	Point2f patternPosition ;
	vector<cv::Point3f> candidateObjectPts;
	vector<cv::Point2f> candidateImagePoints;
	std::vector<std::vector<cv::Point2f> > imagePoints;
protected:

	int projectorWidth;
	int projectorHeight;
	bool ready;
	cv::Size patternSize, checkerBoardSize, addedImageSize;
	float squareSize;
	float checkerSquareSize;
	cv::Mat grayMat;
	cv::Mat distCoeffs;

	std::vector<cv::Mat> boardRotations, boardTranslations;
	std::vector<std::vector<cv::Point3f> > objectPoints;

	float reprojectionError;
	std::vector<float> perViewErrors;
	//camera intrinsic
	cv::Mat cam_distortion_coeffs;
	cv::Mat cam_intrinsics;
	//projector intrinsic
	cv::Mat distortedIntrinsics;
	cv::Mat undistortedIntrinsics;
	CalibrationPattern patternType;
};

#endif /* PROJECTORCALIBRATOR_H_ */
