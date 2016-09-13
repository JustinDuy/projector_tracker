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
void ProjectorCalibration::loadCamIntrinsic(string camcalib, string tag_K, string tag_D){
    cv::FileStorage fs( camcalib, cv::FileStorage::READ );
    if( !fs.isOpened() )
    {
      std::cout << "Failed to open Camera Calibration Data File." << std::endl;
      return ;
    }
    // Loading calibration parameters
    fs[tag_K] >> cam_distortedIntrinsics;
    fs[tag_D] >> cam_distortion_coeffs;
}
bool ProjectorCalibration::loadSetting(string configFile, string tag_w, string tag_h, string tag_pattern_type,
		string tag_pattern_w, string tag_pattern_h, string tag_square_sz, string tag_x, string tag_y,
		string tag_checker_w, string tag_checker_h, string tag_checker_sz,
		string tag_max_repr_error,
		string tag_num_clean, string tag_num_final, string cam_matrix, string tag_k, string tag_d){
	 cv::FileStorage fs( configFile, cv::FileStorage::READ );
	if( !fs.isOpened() )
	{
// 	  std::cout << "Failed to open Setting File." << std::endl;
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

	fs[tag_w] >> projectorWidth;
	std::cout << "projectorWidth : "<< projectorWidth << std::endl;
	fs[tag_h] >> projectorHeight;
	std::cout << "projectorHeight : "<< projectorHeight << std::endl;
	ImagerSize = Size(projectorWidth, projectorHeight);

	float corner_x=0, corner_y=0;
	fs[tag_x] >> corner_x;
	std::cout << "corner_x : "<< corner_x << std::endl;
	fs[tag_y] >> corner_y;
	std::cout << "corner_y : "<< corner_y << std::endl;
	setPatternPosition(corner_x,corner_y);
        
	int patternW, patternH, checkerW, checkerH;
	fs[tag_pattern_w] >> patternW;
	std::cout << "patternW : "<< patternW << std::endl;
	fs[tag_pattern_h] >> patternH;
	std::cout << "patternH : "<< patternH << std::endl;
	patternSize = Size(patternW, patternH);
	
	fs[tag_square_sz] >> squareSize;
	std::cout << "squareSize : "<< squareSize << std::endl;

	fs[tag_checker_w] >> checkerW;
	std::cout << "checkerboard patternW : "<< checkerW << std::endl;
	fs[tag_checker_h] >> checkerH;
	std::cout << "checkerboard patternH : "<< checkerH << std::endl;
        checkerBoardSize = Size(checkerW, checkerH);

	fs[tag_checker_sz] >> checkerSquareSize;
	std::cout << "checker squareSize : "<< checkerSquareSize << std::endl;

	fs[tag_max_repr_error] >> maxReprojectionError;
	std::cout << "maxReprojectionError : "<< maxReprojectionError << std::endl;
	fs[tag_num_clean] >> numBoardsBeforeCleaning;
	std::cout << "numBoardsBeforeCleaning : "<< numBoardsBeforeCleaning << std::endl;
	fs[tag_num_final] >> numBoardsFinalCamera;
	std::cout << "numBoardsFinalCamera : "<< numBoardsFinalCamera << std::endl;
	//reading Kc
	std::cout << "reading camera intrinsic from " << cam_matrix << std::endl;
	loadCamIntrinsic(cam_matrix, tag_k, tag_d);
	return true;
}


void ProjectorCalibration::computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, const vector<cv::Point3f> & objPts, cv::Mat& boardRot, cv::Mat& boardTrans){
cv::solvePnP(objPts, imgPts,
		cam_distortedIntrinsics,
		cam_distortion_coeffs,
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
            cam_imagePoints.erase(cam_imagePoints.begin() + i);
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
void ProjectorCalibration::updateImagePoints(){
    setStaticCandidateImagePoints();
    imagePoints.resize(objectPoints.size(), candidateImagePoints);
}
bool ProjectorCalibration::calibrate() {
	if(size() < 1) {
		cout << "Calibration::calibrate() doesn't have any image data to calibrate from." << endl;
		return false;
	}
	cout << "pose #"<< size() << endl;
	//Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

	int calibFlags = 0;
	float rms = calibrateCamera(objectPoints, imagePoints, ImagerSize, distortedIntrinsics, distCoeffs, boardRotations, boardTranslations, calibFlags);
    /*cv::stereoCalibrate(objectPoints,
                            cam_imagePoints,
                            imagePoints,
                            cam_distortedIntrinsics, cam_distortion_coeffs,
                            distortedIntrinsics, distCoeffs,
                            addedImageSize,
                            rotation3x3, transCamToProj,
                            essentialMatrix, fundamentalMatrix, CV_CALIB_FIX_INTRINSIC);
	cv::Rodrigues(rotation3x3, rotCamToProj);
	*/
	ready = checkRange(distortedIntrinsics) && checkRange(distCoeffs);
	if(!ready) {
		cout <<  "Calibration::calibrate() failed to calibrate the projector" << endl;
	}
	else {
		cout <<  "Calibration::calibrate() succeeded to calibrate the projector" << endl;
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
       // Todo's
        double err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);//imagePoints2
        int n = objectPoints[i].size();
        perViewErrors[i] = sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
        cout <<  "view " << i << " has error of " << perViewErrors[i] << endl;
    }

    reprojectionError = sqrt(totalErr / totalPoints);

    cout << "all views have error of " << reprojectionError << endl;
}
void ProjectorCalibration::save(string filename, bool absolute) const {
	if(!ready){
		cout << "ProjectorCalibration::save() failed, because your calibration isn't ready yet!" << endl;
	}
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "projectorMatrix" << distortedIntrinsics;
	fs << "projectorWidth" << projectorWidth;
	fs << "projectorHeight" << projectorHeight;

	fs << "distCoeffs" << distCoeffs;
	fs << "reprojectionError" << reprojectionError;
	/*fs << "features" << "[";
	for(int i = 0; i < (int)imagePoints.size(); i++) {
		fs << "[:" << imagePoints[i] << "]";
	}
	fs << "]";*/
}
void ProjectorCalibration::drawCircleGrid(const Mat& img, vector<Point2f> checkerPointBuf, vector<Point2f> circlePointBuf){
	// Draw the corners.
        Mat drawImg = img.clone();
        drawChessboardCorners( drawImg, checkerBoardSize, Mat(checkerPointBuf), true );
	for(const auto & p : circlePointBuf) {
            circle(drawImg, p, 5, Scalar( 0, 0, 255), CV_FILLED); 
	}
	cv::imwrite("detected_circle.jpg", drawImg);
	cv::namedWindow( "circlegrid", cv:: WINDOW_NORMAL );// Create a window for display.
	resizeWindow( "circlegrid", 640, 480 );
	// Moving window of circlegrid to see the image at first screen
	moveWindow( "circlegrid", 0, 0 );
	imshow( "circlegrid", drawImg );                   // Show our image inside it.
	waitKey(5);
}
void ProjectorCalibration::drawCheckerBoard(const Mat& img, vector<Point2f> pointBuf){
	// Draw the corners.
        Mat drawImg = img.clone();
	drawChessboardCorners( drawImg, checkerBoardSize, Mat(pointBuf), true );
        cv::imwrite("detected_checkerboard.jpg", drawImg);
	//cv::namedWindow( "checkerboard", cv::WINDOW_AUTOSIZE );// Create a window for display.
	//imshow( "checkerboard", img );                   // Show our image inside it.
	//waitKey(5);
}
bool ProjectorCalibration::add(const Mat& img, const Mat& processedImg,  vector<Point2f>& circlesImgPts) {
    addedImageSize = img.size();

    // find corners
    vector<Point2f> chessImgPts;
    int chessFlags = CV_CALIB_CB_ADAPTIVE_THRESH;// | CV_CALIB_CB_NORMALIZE_IMAGE;
    bool foundCheckerBoard = findChessboardCorners(img, checkerBoardSize, chessImgPts, chessFlags);
    if(foundCheckerBoard){
        cout << "found checker board" << endl;
        //drawCheckerBoard(img, chessImgPts);
        vector<cv::Point2f> circlesImgPts;

        SimpleBlobDetector::Params params;
        params.maxArea = 10e4;
        params.minArea = 10;
        params.minDistBetweenBlobs = 5;
        Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);

        bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, patternSize, circlesImgPts, patternType, blobDetector);
        
        if(bProjectedPatternFound){
            cout << "found circle grid" << endl;
            drawCircleGrid(img,chessImgPts, circlesImgPts);
            vector<cv::Point3f> circlesObjectPts;
            cv::Mat boardRot;
            cv::Mat boardTrans;
            vector<cv::Point3f> checkerObjectPts = createObjectPoints(checkerBoardSize, checkerSquareSize, CHESSBOARD);
            computeCandidateBoardPose(chessImgPts, checkerObjectPts , boardRot, boardTrans);
            backProject(boardRot, boardTrans, circlesImgPts, circlesObjectPts);
            
            //camera points:
            cam_imagePoints.push_back(circlesImgPts);
            
            //projector points
            imagePoints.push_back(candidateImagePoints);
            objectPoints.push_back(circlesObjectPts);
            return true;
        }
    }
    else
        std::cout << "Calibration::add() failed, maybe your patternSize is wrong or the image has poor lighting?" << std::endl;;
    return false;
}
vector<Point3f> ProjectorCalibration::createObjectPoints(cv::Size _patternSize, float _squareSize, CalibrationPattern _patternType) {
    vector<Point3f> corners;
    switch(_patternType) {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for(int i = 0; i < _patternSize.height; i++)
                for(int j = 0; j < _patternSize.width; j++)
                    corners.push_back(Point3f(float(j * _squareSize), float(i * _squareSize), 0));
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            for(int i = 0; i < _patternSize.height; i++)
                for(int j = 0; j < _patternSize.width; j++)
                    corners.push_back(Point3f(float(((2 * j) + (i % 2)) * _squareSize), float(i * _squareSize), 0));
//             break;
    }
    return corners;
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
	//Mat Kinv64 = cam_distortedIntrinsics.inv();
	//use cam_undistorted_intrinsics
	Mat cam_undistorted_intrinsics = getOptimalNewCameraMatrix(cam_distortedIntrinsics, cam_distortion_coeffs, addedImageSize, 1); //fillFrame ? 0 : 1s
        Mat Kinv64 = cam_undistorted_intrinsics.inv();
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
        cout << "generating pattern for projector width and height : " << projectorWidth << "," << projectorHeight << endl;
	out = Mat::zeros(ImagerSize,CV_8UC3);
	for(const auto & p : candidateImagePoints) {
		circle(out, p , 20, Scalar( 255, 0, 0), CV_FILLED); //blue circles
	}
}
void ProjectorCalibration::setStaticCandidateImagePoints(){
    candidateImagePoints.clear();
    Point2f p;
    if(patternType == ASYMMETRIC_CIRCLES_GRID ){
		for(int i = 0; i < patternSize.height; i++) {
			for(int j = 0; j < patternSize.width; j++) {
				p.x = patternPosition.x + float(((2 * j) + (i % 2)) * squareSize);
				p.y = patternPosition.y + float(i * squareSize);
				candidateImagePoints.push_back(p);
			}
		}
    }
    else if(patternType == CIRCLES_GRID || patternType == CHESSBOARD ){
    	for(int i = 0; i < patternSize.height; i++) {
			for(int j = 0; j < patternSize.width; j++) {
				p.x = patternPosition.x + float(j * squareSize);
				p.y = patternPosition.y + float(i * squareSize);
				candidateImagePoints.push_back(p);
			}
		}
    }
}


void ProjectorCalibration::setPatternPosition(float px, float py) {
    patternPosition = Point2f(px, py);
}


