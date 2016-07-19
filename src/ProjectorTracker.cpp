#include <projector_tracker/ProjectorTracker.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;

ProjectorTracker::ProjectorTracker (std::shared_ptr<CameraProjectorInterface> cpi)
:cp_interface(cpi)
{

}
bool ProjectorTracker::loadSetting(std::string matrix_file , std::string tag_useAruco, std::string tag_knownObj, std::string tag_arucoW, std::string tag_arucoH, std::string tag_arucoNu, std::string tag_PatternW, std::string tag_PatternH, std::string tag_Square)
{
    cv::FileStorage fs( matrix_file, cv::FileStorage::READ );
    if( !fs.isOpened() )
    {
      std::cout << "Failed to open Setting File." << std::endl;
      return false;
    }
    // Loading calibration parameters
    int iAruco = 0, iKnownObj =0;
    fs[tag_useAruco] >> iAruco;
    std::cout << "useAruco : "<< iAruco << std::endl;
    useAruco = iAruco != 0;
    fs[tag_knownObj] >> iKnownObj;
    std::cout << "knownObj : "<< iKnownObj << std::endl;
    known3DObj = iKnownObj != 0;

    fs[tag_arucoW] >> arucoW;
    std::cout << "arucoW : "<< arucoW << std::endl;
    fs[tag_arucoH] >> arucoH;
    std::cout << "arucoH : "<< arucoH << std::endl;
    fs[tag_PatternW] >> patternW;
    std::cout << "patternW : "<< patternW << std::endl;
    fs[tag_PatternH] >> patternH;
    fs[tag_arucoNu] >> arucoNu;
    std::cout << "patternH : "<< patternH << std::endl;
    fs[tag_Square] >> squareSize;
    std::cout << "squareSize : "<< squareSize << std::endl;
    return true;
}

// Generates the images needed for shadowMasks computation
void getImagesForShadowMasks (int width, int height, Mat &blackImage, Mat &whiteImage) {
    blackImage = Mat (height, width, CV_8UC3, Scalar (0, 0, 0));
    whiteImage = Mat (height, width, CV_8UC3, Scalar (255, 255, 255));
}

// Computes the required number of pattern images
void computeNumberOfImages (size_t width, size_t height, size_t &numOfColImgs, size_t &numOfRowImgs, size_t &numOfPatternImages) {
    numOfColImgs = (size_t) ceil (log (double (width)) / log (2.0));
    numOfRowImgs = (size_t) ceil (log (double (height)) / log (2.0));
    numOfPatternImages = 2 * numOfColImgs + 2 * numOfRowImgs;
}

int getPatternImageNum (int width, int height) {
    size_t numOfPatterns = -1;
    size_t nCols, nRows;
    computeNumberOfImages (width, height, nRows, nCols, numOfPatterns);
    return (int) numOfPatterns;
}

vector<Mat> ProjectorTracker::getPatternImages () {
    int width = cp_interface->getProjectorCalibration().width;
    int height = cp_interface->getProjectorCalibration().height;
    if(useAruco){//USE ARUCO MARKERS
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(arucoW, arucoH, 0.04f, 0.02f, dictionary);
        cv::Mat boardImage;
        board->draw( cv::Size(width, height), boardImage, 10, 1 );
        vector<Mat> ret;
        for(int i = 0; i < arucoNu ; i++)
            ret.push_back(boardImage);//projecting the same charuco markers on different boards
        imwrite("boadaruco.jpg", boardImage);
        return ret;
    }
    else
    {//USE GRAY CODE
        size_t numOfPatternImages ;
        size_t numOfRowImgs;
        size_t numOfColImgs;
        computeNumberOfImages (width, height, numOfColImgs, numOfRowImgs, numOfPatternImages);

        vector<Mat> ret;
        ret.resize (numOfPatternImages);

        for (size_t i = 0; i < numOfPatternImages; i++) {
            ret[i] = Mat (height, width, CV_8U);
        }

        uchar flag = 0;

        for (int j = 0; j < width; j++) { // rows loop
            int rem = 0, num = j, prevRem = j % 2;

            for (size_t k = 0; k < numOfColImgs; k++) { // images loop
                num = num / 2;
                rem = num % 2;

                if ( (rem == 0 && prevRem == 1) || (rem == 1 && prevRem == 0)) {
                    flag = 1;
                } else {
                    flag = 0;
                }

                for (int i = 0; i < height; i++) { // rows loop

                    uchar pixel_color = (uchar) flag * 255;

                    ret[2 * numOfColImgs - 2 * k - 2].at<uchar> (i, j) = pixel_color;

                    if (pixel_color > 0)
                        pixel_color = (uchar) 0;
                    else
                        pixel_color = (uchar) 255;

                    ret[2 * numOfColImgs - 2 * k - 1].at<uchar> (i, j) = pixel_color;   // inverse
                }

                prevRem = rem;
            }
        }

        for (int i = 0; i < height; i++) { // rows loop
            int rem = 0, num = i, prevRem = i % 2;

            for (size_t k = 0; k < numOfRowImgs; k++) {
                num = num / 2;
                rem = num % 2;

                if ( (rem == 0 && prevRem == 1) || (rem == 1 && prevRem == 0)) {
                    flag = 1;
                } else {
                    flag = 0;
                }

                for (int j = 0; j < width; j++) {
                    uchar pixel_color = (uchar) flag * 255;
                    ret[2 * numOfRowImgs - 2 * k + 2 * numOfColImgs - 2].at<uchar> (i, j) = pixel_color;

                    if (pixel_color > 0)
                        pixel_color = (uchar) 0;
                    else
                        pixel_color = (uchar) 255;

                    ret[2 * numOfRowImgs - 2 * k + 2 * numOfColImgs - 1].at<uchar> (i, j) = pixel_color;
                }

                prevRem = rem;
            }
        }

        // Generate the all-white and all-black images needed for shadows mask computation
        Mat white;
        Mat black;
        getImagesForShadowMasks (width, height, black, white);
        ret.push_back (black);
        ret.push_back (white);
        return ret;
    }
}

// Computes the shadows occlusion where we cannot reconstruct the model
void computeShadowMask (const Mat blackImage, const Mat whiteImage, double blackThreshold, Mat &shadowMask) {
    int cam_width = whiteImage.cols;
    int cam_height = blackImage.rows;
    shadowMask = Mat (cam_height, cam_width, CV_8U);
    for (int i = 0; i < cam_height; i++) {
        for (int j = 0; j < cam_width; j++) {
            double white = whiteImage.at<uchar> (i, j) ;
            double black = blackImage.at<uchar> (i, j) ;
            double diff = abs (white - black) ;

            //cout << diff << ", ";
            if (diff > blackThreshold) {
                shadowMask.at<uchar> (i, j)  = (uchar) 1;
            } else {
                shadowMask.at<uchar> (i, j) = (uchar) 0;
            }
        }
    }
}

// Converts a gray code sequence (~ binary number) to a decimal number
int grayToDec (const vector<uchar> &gray) {
    int dec = 0;

    uchar tmp = gray[0];

    if (tmp)
        dec += (int) pow ( (float) 2, int (gray.size() - 1));

    for (int i = 1; i < (int) gray.size(); i++) {
        // XOR operation
        tmp = tmp ^ gray[i];

        if (tmp)
            dec += (int) pow ( (float) 2, int (gray.size() - i - 1));
    }

    return dec;
}

// For a (x,y) pixel of the camera returns the corresponding projector's pixel
bool getProjPixel (const vector<Mat> &patternImages, size_t width, size_t height, int x, int y, Point &projPix) {
    vector<uchar> grayCol;
    vector<uchar> grayRow;
    bool error = false;
    int xDec, yDec;
    size_t numOfColImgs, numOfRowImgs, numOfPatternImages;
    computeNumberOfImages (width, height, numOfColImgs, numOfRowImgs, numOfPatternImages);
    // process column images
    for (size_t count = 0; count < numOfColImgs; count++) {
        // get pixel intensity for regular pattern projection and its inverse
        double val1 = patternImages[count * 2].at<uchar> (Point (x, y));
        double val2 = patternImages[count * 2 + 1].at<uchar> (Point (x, y));
        double diff = abs (val1 - val2) ;

        // check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
        if (diff < DEFAULT_WHITE_THRESHOLD)
            error = true;

        // determine if projection pixel is on or off
        if (val1 > val2)
            grayCol.push_back (1);
        else
            grayCol.push_back (0);
    }

    xDec = grayToDec (grayCol);

    // process row images
    for (size_t count = 0; count < numOfRowImgs; count++) {
        // get pixel intensity for regular pattern projection and its inverse
        double val1 = patternImages[count * 2 + numOfColImgs * 2].at<uchar> (Point (x, y));
        double val2 = patternImages[count * 2 + numOfColImgs * 2 + 1].at<uchar> (Point (x, y));

        // check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
        if (abs (val1 - val2) < DEFAULT_WHITE_THRESHOLD)
            error = true;

        // determine if projection pixel is on or off
        if (val1 > val2)
            grayRow.push_back (1);
        else
            grayRow.push_back (0);
    }

    yDec = grayToDec (grayRow);

    if ( (yDec >= height || xDec >= width)) {
        error = true;
    }
    projPix.x = xDec;
    projPix.y = yDec;
    return error;
}
void ProjectorTracker::saveExtrinsics(cv::Mat rotCamToProj, cv::Mat transCamToProj, std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "Rotation_Vector" << rotCamToProj;
    fs << "Translation_Vector" << transCamToProj;
}
bool ProjectorTracker::backProject(const cv::Mat&  K, const cv::Mat& boardRot64,
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
        Mat Kinv64 = K.inv();
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
bool ProjectorTracker::findBoard(const cv::Mat& img, vector<cv::Point2f>& pointBuf, bool refine) {
    bool found=false;
    // no CV_CALIB_CB_FAST_CHECK, because it breaks on dark images (e.g., dark IR images from kinect)
    int chessFlags = CV_CALIB_CB_ADAPTIVE_THRESH;// | CV_CALIB_CB_NORMALIZE_IMAGE;
    found = findChessboardCorners(img, cv::Size(patternW,patternH), pointBuf, chessFlags);

    // improve corner accuracy
    if(found) {
        cv::Mat grayMat;
        if(img.type() != CV_8UC1) {
            cv::cvtColor(img, grayMat, CV_RGB2GRAY);
        } else {
            grayMat = img;
        }

        if(refine) {
            // the 11x11 dictates the smallest image space square size allowed
            // in other words, if your smallest square is 11x11 pixels, then set this to 11x11
            cornerSubPix(grayMat, pointBuf, cv::Size(1,1),  cv::Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
        }
    }

    return found;
}
void ProjectorTracker::computeCandidateBoardPose(const vector<cv::Point3f> & img3DPts, const vector<cv::Point2f> & imgPts,
                                                 const cv::Mat& K, const cv::Mat& distCoeffs, cv::Mat& boardRot, cv::Mat& boardTrans){
    cv::solvePnP(img3DPts, imgPts,
                 K,
                 distCoeffs,
                 boardRot, boardTrans);
}


void ProjectorTracker::findCorrespondence_Aruco(const std::vector<CameraProjectorImagePair>& cp_images ,std::vector<std::vector<cv::Point2f> >& camPixels_boards, std::vector<std::vector<cv::Point2f> >& projPixels_boards)
{
    //int ret = 0;
    for(int j=0; j < arucoNu; j++){
        Mat projected = cp_images[j].projected;
        Mat copyProjected;
        projected.copyTo(copyProjected);
        Mat acquired = cp_images[j].acquired;
        Mat copyAcquired;
        acquired.copyTo(copyAcquired);
        std::vector<cv::Point2f> camPixels;
        std::vector<cv::Point2f> projPixels;
        //define Aruco Board
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr< cv::aruco::CharucoBoard > board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters > params = cv::aruco::DetectorParameters::create();
        params->doCornerRefinement = false;
        //detect Aruco corners on projector image
        std::vector<int> proj_ids;
        std::vector<std::vector<cv::Point2f> > proj_corners;
        cv::aruco::detectMarkers(projected, dictionary, proj_corners, proj_ids, params);
        std::vector<cv::Point2f> proj_charucoCorners;
        std::vector<int> proj_charucoIds;
        // if at least one marker detected
        if (proj_ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(copyProjected, proj_corners, proj_ids);
            cv::aruco::interpolateCornersCharuco(proj_corners, proj_ids, projected, board, proj_charucoCorners, proj_charucoIds);
            // if at least one charuco corner detected
            if(proj_charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(copyProjected, proj_charucoCorners, proj_charucoIds, cv::Scalar(255, 0, 0));
        }

        //cv::imshow("aruco projected", copyProjected);
        //cv::waitKey(1000);
        //cout << "saving arucoprojected.jpeg" << endl;
        //imwrite("arucoprojected.jpeg",copyProjected);

        //detect Aruco corners on camera image
        std::vector<int> cam_ids;
        std::vector<std::vector<cv::Point2f> > cam_corners;
        cv::aruco::detectMarkers(acquired, dictionary, cam_corners, cam_ids, params);
        std::vector<cv::Point2f> cam_charucoCorners;
        std::vector<int> cam_charucoIds;
        // if at least one marker detected
        if (cam_ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(copyAcquired, cam_corners, cam_ids);
            cv::aruco::interpolateCornersCharuco(cam_corners, cam_ids, acquired, board, cam_charucoCorners, cam_charucoIds);
            // if at least one charuco corner detected
            if(cam_charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(copyAcquired, cam_charucoCorners, cam_charucoIds, cv::Scalar(255, 0, 0));
        }

        //cv::imshow("aruco aquired", copyAcquired);
        //cv::waitKey(1000);
        //cout << "saving arucoacquired.jpeg" << endl;
        //imwrite("arucoacquired.jpeg",copyAcquired);
        //matching project pixels and cam pixels based on aruco id:
        for(int i = 0; i < cam_charucoIds.size(); i++){
            for(int j = 0; j < proj_charucoIds.size(); j++){
                if(cam_charucoIds[i] == proj_charucoIds[j]){
                    camPixels.push_back(cam_charucoCorners[i]);
                    projPixels.push_back(proj_charucoCorners[j]);
                }
            }
        }
        //ret = camPixels.size();
        //cout << "matching list of size " << ret << endl;
        camPixels_boards.push_back(camPixels);
        projPixels_boards.push_back(projPixels);
    }
}
int ProjectorTracker::findCorrespondence_GrayCode(const std::vector<CameraProjectorImagePair>& cp_images, std::vector<cv::Point2f>& camPixels, std::vector<cv::Point2f>& projPixels){
    //use graycode to find correspondence
    Mat blackImage, whiteImage;
    int seq_length = cp_images.size();
    blackImage = cp_images[seq_length - 2].acquired;
    whiteImage = cp_images[seq_length - 1].acquired;
    vector<Mat> camera_images;
    for(int i=0;i< seq_length; i++){
        camera_images.push_back(cp_images[i].acquired);
    }
    // Computing shadows mask
    Mat shadowMask;
    imwrite("black.jpeg",blackImage);
    imwrite("white.jpeg",whiteImage);
    computeShadowMask (blackImage, whiteImage, DEFAULT_BLACK_THRESHOLD, shadowMask);
    imwrite("shadow.jpeg",shadowMask);

    //dim identify:
    int cam_width = cp_interface->getCameraCalibration().width;
    int cam_height = cp_interface->getCameraCalibration().height;
    int proj_width = cp_interface->getProjectorCalibration().width;
    int proj_height = cp_interface->getProjectorCalibration().height;

    Point projPixel;
    for (int i = 0; i < cam_width; i++) {
        for (int j = 0; j < cam_height; j++) {
            //if the pixel is not shadowed, reconstruct
            if (shadowMask.at<uchar> (j, i)) {
                //for a (x,y) pixel of the camera returns the corresponding projector pixel by calculating the decimal number
                bool error = getProjPixel (camera_images, proj_width, proj_height, i, j, projPixel);
                if (error)
                {
                    continue;
                }
                else
                {
                    camPixels.push_back (Point (i, j));
                    projPixels.push_back (projPixel);
                    //visualize projector correspondence on camera image
                    circle (whiteImage, Point (i , j), 2, (255, 0, 0), 0);
                }
            }
        }
    }
    imwrite("correspondence.jpeg",whiteImage);

}
void ProjectorTracker::computeRt_knownObjectPoints(const std::vector<CameraProjectorImagePair>& cp_images,
                                                   std::vector<std::vector<cv::Point2f> > camPxls,
                                                   std::vector<std::vector<cv::Point2f> > projPxls,
                                                   cv::Mat& transCamToProj ){
    //this function is to compute transformation using known checkboard:
    std::vector<cv::Point3f> aruco3DPoints;
    std::vector<cv::Point2f> camArucoPoints;
    std::vector<cv::Point2f> projArucoPoints;
    cv::Mat projectorMatrix     = cp_interface->getProjectorCalibration().intrinsics;
    cv::Mat projectorDistCoeffs = cp_interface->getProjectorCalibration().distortion_coeffs;
    cv::Mat cameraMatrix        = cp_interface->getCameraCalibration().intrinsics;
    cv::Mat cameraDistCoeffs    = cp_interface->getCameraCalibration().distortion_coeffs;
    cv::Size camImgSize = cp_images[0].acquired.size();
    for(int board = 0; board < camPxls.size(); board++){
        Mat cam_acquired = cp_images[board].acquired;
        for(int i =0; i< camPxls[board].size(); i++){
            camArucoPoints.push_back(camPxls[board][i]);
            projArucoPoints.push_back(projPxls[board][i]);
        }
        //generate 3D object points (corners of board) (respect to board origin)
        vector<Point3f> candidateObjectPts;
        for(int i = 0; i < patternH; i++) {
            for(int j = 0; j < patternW; j++) {
                candidateObjectPts.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
            }
        }
        //detect board corners on camera image
        vector<Point2f> patternPoints;
        findBoard(cam_acquired, patternPoints, false);
        //compute camera extrinsic based on known 3D points and detected 2D points
        cv::Mat boardR;
        cv::Mat boardT;
        computeCandidateBoardPose(candidateObjectPts, patternPoints, cameraMatrix, cameraDistCoeffs, boardR, boardT);
        //backproject camera's aruco 2D points to get camera's 3D aruco points
        backProject(cameraMatrix, boardR, boardT, camPxls[board], aruco3DPoints);
    }
    //stereoCalibrate btw camera and projector using aruco points:
    cv::Mat fundamentalMatrix, essentialMatrix;
    cv::Mat rotation3x3;
    cv::stereoCalibrate(aruco3DPoints,
                        camArucoPoints,
                        projArucoPoints,
                        cameraMatrix, cameraDistCoeffs,
                        projectorMatrix, projectorDistCoeffs,
                        camImgSize,
                        rotation3x3, transCamToProj,
                        essentialMatrix, fundamentalMatrix);
    cv::Mat rotCamToProj;
    cv::Rodrigues(rotation3x3, rotCamToProj);
    saveExtrinsics(rotCamToProj, transCamToProj, "cam2proj_knownScenceObj.yml");
}
bool ProjectorTracker::computeRt_unknownObjectPoints( std::vector<std::vector<cv::Point2f> >& camPixels_boards, std::vector<std::vector<cv::Point2f> >& projPixels_boards, cv::Mat& ret){
    //compute transformation matrix from camera to projector
    cv::Mat projectorMatrix     = cp_interface->getProjectorCalibration().intrinsics;
    //cv::Mat projectorDistCoeffs = cp_interface->getProjectorCalibration().distortion_coeffs;
    cv::Mat cameraMatrix        = cp_interface->getCameraCalibration().intrinsics;
    //cv::Mat cameraDistCoeffs    = cp_interface->getCameraCalibration().distortion_coeffs;
    std::vector<cv::Point2f> camPixels;
    std::vector<cv::Point2f> projPixels;
    //merge correspondence into same vectors
    for(int board= 0; board < camPixels_boards.size(); board++ ){
        for(int i = 0; i< camPixels_boards[board].size(); i ++){
            camPixels.push_back(camPixels_boards[board][i]);
            projPixels.push_back(projPixels_boards[board][i]);
        }
    }
    if (camPixels.size() == projPixels.size() && camPixels.size() > 9) {
        Mat F = findFundamentalMat (camPixels, projPixels, FM_RANSAC, 3, 0.99);
        Mat E = projectorMatrix.t() * F * cameraMatrix;
        //Perform SVD on E
        SVD decomp = SVD (E);

        //U
        Mat U = decomp.u;

        //S
        Mat S (3, 3, CV_64F, Scalar (0));
        S.at<double> (0, 0) = decomp.w.at<double> (0, 0);
        S.at<double> (1, 1) = decomp.w.at<double> (0, 1);
        S.at<double> (2, 2) = decomp.w.at<double> (0, 2);

        //Vt
        Mat Vt = decomp.vt;

        //W
        Mat W (3, 3, CV_64F, Scalar (0));
        W.at<double> (0, 1) = -1;
        W.at<double> (1, 0) = 1;
        W.at<double> (2, 2) = 1;

        Mat Wt (3, 3, CV_64F, Scalar (0));
        Wt.at<double> (0, 1) = 1;
        Wt.at<double> (1, 0) = -1;
        Wt.at<double> (2, 2) = 1;

        Mat R1 = U * W * Vt;
        //Mat R2 = U * Wt * Vt;
        Mat u1 = U.col (2);
        //Mat u2 = -U.col (2);
        //4 candidates
        //cout << "computed rotation, translation: " << endl;
        //cout << R1 << "," << u1 << endl;

        //save R1, u1 to "cam_proj_trans.yaml"
        saveExtrinsics(R1, u1, "../data/cam_proj_trans.yaml");

        ret.at<double> (3, 3) = 1;
        for(int i = 0; i < 3; i++){
            for(int j =0 ; j < 3; j++){
                ret.at<double> (j,i) = R1.at<double> (j,i);
            }
        }
        for(int i=0;i<3;i++)
            ret.at<double> (i,0) = u1.at<double> (0,i);
        ret.at<double> (3,3) = 1;
        for(int i=0;i<3;i++)
            ret.at<double> (3,i) = 0;
        return true;
    }
    else
    {
        //cerr << "correspodence lists size mismatched or not enough correspondences" << endl;
        return false;
    }
}
Mat ProjectorTracker::computeRelativePosition (const std::vector<CameraProjectorImagePair>& cp_images) {
    //this function computes transformation matrix from camera to projector
    Mat rt (4, 4, CV_64F, Scalar (0));
    vector<vector<Point2f> > camPixels;
    vector<vector<Point2f> > projPixels;
    Mat projected = cp_images[0].projected;
    Mat copyProjected;
    projected.copyTo(copyProjected);
    Mat acquired = cp_images[0].acquired;
    Mat copyAcquired;
    acquired.copyTo(copyAcquired);

    //establish correspondent pair of camera pixel & projector pixel
    if(useAruco)
    {
        findCorrespondence_Aruco(cp_images, camPixels, projPixels );
    }
    /*else
    {
        findCorrespondence_GrayCode(cp_images, camPixels, projPixels);
    }*/
    if(!known3DObj)
        computeRt_unknownObjectPoints(camPixels,projPixels, rt);
    else
        computeRt_knownObjectPoints(cp_images, camPixels, projPixels, rt);

    return rt;
}
