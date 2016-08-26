/*
 * ProjectorCalibration.cpp
 *
 *  Created on: Aug 24, 2016
 *      Author: justin
 */
#include <projector_tracker/ProjectorCalibration.h>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;
bool ProjectorCalibration::loadSetting(string configFile, string tag_pattern_type,
		string tag_pattern_w, string tag_pattern_h, string tag_square_sz,
		string tag_checker_w, string tag_checker_h, string tag_checker_sz,
		string tag_max_repr_error,
		string tag_num_clean, string tag_num_final){
	 cv::FileStorage fs( configFile, cv::FileStorage::READ );
	if( !fs.isOpened() )
	{
	  std::cout << "Failed to open Setting File." << std::endl;
	  return false;
	}
	// Loading calibration parameters
	int type = -1;
	fs[tag_pattern_type] >> type;
	switch(type ) {
		case 0:
			patternType = CHESSBOARD;
			break;
		case 1:
			patternType = CIRCLES_GRID;
			break;
		case 2:
			patternType = ASYMMETRIC_CIRCLES_GRID;
	}

	int patternW, patternH, checkerW, checkerH;
	fs[tag_pattern_w] >> patternW;
	std::cout << "patternW : "<< patternW << std::endl;
	fs[tag_pattern_h] >> patternH;
	std::cout << "patternH : "<< patternH << std::endl;
	patternSize = Size(patternW, patternH);
	checkerBoardSize = Size(checkerW, checkerH);

	fs[tag_square_sz] >> squareSize;
	std::cout << "squareSize : "<< squareSize << std::endl;

	fs[tag_pattern_w] >> checkerW;
	std::cout << "checkerboard patternW : "<< checkerW << std::endl;
	fs[tag_pattern_h] >> checkerH;
	std::cout << "checkerboard patternH : "<< checkerH << std::endl;

	fs[tag_checker_sz] >> checkerSquareSize;
	std::cout << "checker squareSize : "<< checkerSquareSize << std::endl;

	fs[tag_max_repr_error] >> maxReprojectionError;
	std::cout << "maxReprojectionError : "<< maxReprojectionError << std::endl;
	fs[tag_num_clean] >> numBoardsBeforeCleaning;
	std::cout << "numBoardsBeforeCleaning : "<< numBoardsBeforeCleaning << std::endl;
	fs[tag_num_final] >> numBoardsFinalCamera;
	std::cout << "numBoardsFinalCamera : "<< numBoardsFinalCamera << std::endl;
	return true;
}


void ProjectorCalibration::computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, const vector<cv::Point3f> & objPts, cv::Mat& boardRot, cv::Mat& boardTrans){
cv::solvePnP(objPts, imgPts,
			 distortedIntrinsics,
			 distCoeffs,
			 boardRot, boardTrans);
}
int ProjectorCalibration::size() const {
    return imagePoints.size();
}
bool ProjectorCalibration::clean() {
    int removed = 0;
    for(int i = size() - 1; i >= 0; i--) {
        if(getReprojectionError(i) > maxReprojectionError) {
            objectPoints.erase(objectPoints.begin() + i);
            imagePoints.erase(imagePoints.begin() + i);
            removed++;
        }
    }
    if(size() > 0) {
        if(removed > 0) {
            return calibrate();
        } else {
            return true;
        }
    } else {
        cout << "Calibration::clean() removed the last object/image point pair" << endl;
        return false;
    }
}
float ProjectorCalibration::getReprojectionError() const {
    return reprojectionError;
}
float ProjectorCalibration::getReprojectionError(int i) const {
    return perViewErrors[i];
}

void ProjectorCalibration::updateImagePoints() {
    vector<Point2f> points = createImagePoints(patternSize, squareSize, patternType);
    imagePoints.resize(objectPoints.size(), points);
}
vector<Point2f> ProjectorCalibration::createImagePoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType) {
    vector<Point2f> corners;
    switch(patternType) {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for(int i = 0; i < patternSize.height; i++)
                for(int j = 0; j < patternSize.width; j++)
                    corners.push_back(Point2f(float(j * squareSize), float(i * squareSize)));
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            for(int i = 0; i < patternSize.height; i++)
                for(int j = 0; j < patternSize.width; j++)
                    corners.push_back(Point2f(float(((2 * j) + (i % 2)) * squareSize), float(i * squareSize)));
            break;
    }
    return corners;
}
bool ProjectorCalibration::calibrate() {
	if(size() < 1) {
		cout << "Calibration::calibrate() doesn't have any image data to calibrate from." << endl;
		return false;
	}
	cout << "pose #"<< size() << endl;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

    updateImagePoints();

	int calibFlags = 0;
	float rms = calibrateCamera(objectPoints, imagePoints, addedImageSize, distortedIntrinsics, distCoeffs, boardRotations, boardTranslations, calibFlags);
	ready = checkRange(cameraMatrix) && checkRange(distCoeffs);

	if(!ready) {
		cout <<  "Calibration::calibrate() failed to calibrate the camera" << endl;
	}
	else {
		cout <<  "Calibration::calibrate() succeeded to calibrate the camera" << endl;
	}
	updateReprojectionError();
	updateUndistortion();

	return ready;
}
/*void CameraCalibration::setFillFrame(bool fillFrame) {
    this->fillFrame = fillFrame;
}*/
void ProjectorCalibration::updateUndistortion() {
    undistortedIntrinsics = getOptimalNewCameraMatrix(distortedIntrinsics, distCoeffs, addedImageSize, 1); //fillFrame ? 0 : 1s
    //initUndistortRectifyMap(distortedIntrinsics.getCameraMatrix(), distCoeffs, Mat(), undistortedCameraMatrix, distortedIntrinsics.getImageSize(), CV_16SC2, undistortMapX, undistortMapY);
    //undistortedIntrinsics.setup(undistortedCameraMatrix, distortedIntrinsics.getImageSize());
}
bool ProjectorCalibration::isReady(){
    return ready;
}
void ProjectorCalibration::updateReprojectionError() {
    vector<Point2f> imagePoints2;
    int totalPoints = 0;
    double totalErr = 0;

    perViewErrors.clear();
    perViewErrors.resize(objectPoints.size());

    for(int i = 0; i < (int)objectPoints.size(); i++) {
        projectPoints(Mat(objectPoints[i]), boardRotations[i], boardTranslations[i], distortedIntrinsics, distCoeffs, imagePoints2);
        double err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = objectPoints[i].size();
        perViewErrors[i] = sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
        cout <<  "view " << i << " has error of " << perViewErrors[i] << endl;;
    }

    reprojectionError = sqrt(totalErr / totalPoints);

    cout << "all views have error of " << reprojectionError << endl;;
}
void ProjectorCalibration::save(string filename, bool absolute) const {
	if(!ready){
		cout << "Calibration::save() failed, because your calibration isn't ready yet!" << endl;
	}
	FileStorage fs(filename, FileStorage::WRITE);
	cv::Size imageSize = addedImageSize;
	fs << "cameraMatrix" << distortedIntrinsics;
	fs << "imageSize_width" << imageSize.width;
	fs << "imageSize_height" << imageSize.height;

	fs << "distCoeffs" << distCoeffs;
	fs << "reprojectionError" << reprojectionError;
	fs << "features" << "[";
	for(int i = 0; i < (int)imagePoints.size(); i++) {
		fs << "[:" << imagePoints[i] << "]";
	}
	fs << "]";
}
void ProjectorCalibration::drawCheckerBoard(Mat img, vector<Point2f> pointBuf){
	// Draw the corners.
	drawChessboardCorners( img, patternSize, Mat(pointBuf), true );
	cv::namedWindow( "checkerboard", cv::WINDOW_AUTOSIZE );// Create a window for display.
	imshow( "checkerboard", img );                   // Show our image inside it.
	waitKey(5);
}
bool ProjectorCalibration::add(Mat img, Mat processedImg,  vector<Point2f>& circlesImgPts) {
    addedImageSize = img.size();

    // find corners
    vector<Point2f> chessImgPts;
	int chessFlags = CV_CALIB_CB_ADAPTIVE_THRESH;// | CV_CALIB_CB_NORMALIZE_IMAGE;
	bool foundCheckerBoard = findChessboardCorners(img, checkerBoardSize, chessImgPts, chessFlags);
    if(foundCheckerBoard){
		vector<cv::Point2f> circlesImgPts;

		SimpleBlobDetector::Params params;
		params.maxArea = 10e4;
		params.minArea = 10;
		params.minDistBetweenBlobs = 5;
		Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);

		bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, patternSize, circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID, blobDetector);

		if(bProjectedPatternFound){

			vector<cv::Point3f> circlesObjectPts;
			cv::Mat boardRot;
			cv::Mat boardTrans;
			computeCandidateBoardPose(chessImgPts, createObjectPoints(checkerBoardSize, checkerSquareSize, CHESSBOARD), boardRot, boardTrans);
			backProject(boardRot, boardTrans, circlesImgPts, circlesObjectPts);

			imagePoints.push_back(candidateImagePoints);
			objectPoints.push_back(circlesObjectPts);
			return true;
		}
    }
    else
        std::cout << "Calibration::add() failed, maybe your patternSize is wrong or the image has poor lighting?" << std::endl;;
    return false;
}
vector<Point3f> ProjectorCalibration::createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType) {
    vector<Point3f> corners;
    switch(patternType) {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for(int i = 0; i < patternSize.height; i++)
                for(int j = 0; j < patternSize.width; j++)
                    corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            for(int i = 0; i < patternSize.height; i++)
                for(int j = 0; j < patternSize.width; j++)
                    corners.push_back(Point3f(float(((2 * j) + (i % 2)) * squareSize), float(i * squareSize), 0));
            break;
    }
    return corners;
}
bool ProjectorCalibration::findBoard(Mat img, vector<Point2f>& pointBuf) {
	bool found=false;
	int flags = (patternType == CIRCLES_GRID ? CALIB_CB_SYMMETRIC_GRID : CALIB_CB_ASYMMETRIC_GRID); // + CALIB_CB_CLUSTERING
	found = findCirclesGrid(img, patternSize, pointBuf, flags);
	return found;
}



bool ProjectorCalibration::backProject(const cv::Mat& boardRot64,
								const cv::Mat& boardTrans64,
								const vector<cv::Point2f>& imgPt,
								vector<cv::Point3f>& worldPt) {
if( imgPt.size() == 0 ) {
	return false;
}
else
{
	cv::Mat imgPt_h = cv::Mat::zeros(3, imgPt.size(), CV_32F);
	for( int h=0; h<imgPt.size(); ++h ) {
		imgPt_h.at<float>(0,h) = imgPt[h].x;
		imgPt_h.at<float>(1,h) = imgPt[h].y;
		imgPt_h.at<float>(2,h) = 1.0f;
	}
	Mat Kinv64 = undistortedIntrinsics.inv();
	Mat Kinv,boardRot,boardTrans;
	Kinv64.convertTo(Kinv, CV_32F);
	boardRot64.convertTo(boardRot, CV_32F);
	boardTrans64.convertTo(boardTrans, CV_32F);

	// Transform all image points to world points in camera reference frame
	// and then into the plane reference frame
	Mat worldImgPt = Mat::zeros( 3, imgPt.size(), CV_32F );
	Mat rot3x3;
	Rodrigues(boardRot, rot3x3);

	Mat transPlaneToCam = rot3x3.inv()*boardTrans;

	for( int i=0; i<imgPt.size(); ++i ) {
		Mat col = imgPt_h.col(i);
		Mat worldPtcam = Kinv*col;
		Mat worldPtPlane = rot3x3.inv()*(worldPtcam);

		float scale = transPlaneToCam.at<float>(2)/worldPtPlane.at<float>(2);
		Mat worldPtPlaneReproject = scale*worldPtPlane-transPlaneToCam;

		cv::Point3f pt;
		pt.x = worldPtPlaneReproject.at<float>(0);
		pt.y = worldPtPlaneReproject.at<float>(1);
		pt.z = 0;
		worldPt.push_back(pt);
	}
}
return true;
}

void ProjectorCalibration::getPattern(Mat& out){
	if(candidateImagePoints.size() == 0) setStaticCandidateImagePoints();
	out = Mat::zeros(projectorWidth, projectorHeight, CV_8UC3);
	for(const auto & p : candidateImagePoints) {
		circle(out, p, 20, Scalar( 255, 0, 0), CV_FILLED); //blue circles
	}
}
void ProjectorCalibration::setStaticCandidateImagePoints(){
    candidateImagePoints.clear();
    Point2f p;
    for(int i = 0; i < patternSize.height; i++) {
        for(int j = 0; j < patternSize.width; j++) {
            p.x = patternPosition.x + float(((2 * j) + (i % 2)) * squareSize);
            p.y = patternPosition.y + float(i * squareSize);
            candidateImagePoints.push_back(p);
        }
    }
}


void ProjectorCalibration::setPatternPosition(float px, float py) {
    patternPosition = Point2f(px, py);
}


