/*
 * CameraCalibrator.h
 *
 *  Created on: Aug 24, 2016
 *      Author: justin
 */

#ifndef CAMERACALIBRATOR_H_
#define CAMERACALIBRATOR_H_
#include <projector_tracker/CameraProjectorInterface.h>
#include <opencv2/core.hpp>
using namespace std;
#pragma mark - CameraCalibration
enum CalibrationPattern {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};
class CameraCalibration {

public:
	void computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans);
	bool backProject(const cv::Mat& boardRot64, const cv::Mat& boardTrans64,
					 const vector<cv::Point2f>& imgPt,
					 vector<cv::Point3f>& worldPt);
	void setupCandidateObjectPoints();
	vector<cv::Point3f> getCandidateObjectPoints() { return candidateObjectPts; }
	bool add(cv::Mat img);
	bool calibrate();
	bool findBoard(cv::Mat img, std::vector<cv::Point2f> &pointBuf, bool refine = true);
	void save(std::string filename, bool absolute = false) const;
	int size() const;
	bool clean(float minReprojectionError = 2.f);
	float getReprojectionError() const;
	float getReprojectionError(int i) const;
	void updateReprojectionError();
	void updateObjectPoints() ;
	bool isReady();
	void updateUndistortion();
	static std::vector<cv::Point3f> createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType);
private:
	vector<cv::Point3f> candidateObjectPts;
	std::vector<std::vector<cv::Point2f> > imagePoints;
protected:
	//bool fillFrame;
	bool ready;
	cv::Size patternSize, addedImageSize;
	float squareSize;
	cv::Mat grayMat;
	cv::Mat distCoeffs;

	std::vector<cv::Mat> boardRotations, boardTranslations;
	std::vector<std::vector<cv::Point3f> > objectPoints;

	float reprojectionError;
	std::vector<float> perViewErrors;
	cv::Mat distortedIntrinsics;
	cv::Mat undistortedIntrinsics;
	CalibrationPattern patternType;
};

#endif /* CAMERACALIBRATOR_H_ */
