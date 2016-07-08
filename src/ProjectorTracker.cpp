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

vector<Mat> ProjectorTracker::getPatternImages (int width, int height, bool useAruco) {
    if(useAruco){//USE ARUCO MARKERS
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::CharucoBoard board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
        cv::Mat boardImage;
        board.draw( cv::Size(width, height), boardImage, 10, 1 );
        vector<Mat> ret;
        ret.push_back(boardImage);
    }
    else
    {//USE GRAY CODE
        if(cp_interface->getProjectorCalibration().width != width ||
            cp_interface->getProjectorCalibration().height != height)
        {
            std::cerr << "Projector Resolution hasn't been calibrated!" << std::endl;
        }

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
Mat ProjectorTracker::computeRelativePosition (const std::vector<CameraProjectorImagePair>& cp_images, bool useAruco) {
    // output : camera-projector stereo matrix R, u
    //establish correspondent pair of camera pixel & projector pixel
    vector<Point> camPixels;
    vector<Point> projPixels;
    if(useAruco)
    {
        //cv::namedWindow("aruco aquired", WINDOW_AUTOSIZE);
        //cv::namedWindow("aruco projected", WINDOW_AUTOSIZE);
        //define Aruco Board
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::CharucoBoard board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
        cv::aruco::DetectorParameters params;
        params.doCornerRefinement = false;

        Mat projected = cp_images[0].projected;
        Mat acquired = cp_images[0].acquired;
        Mat copyProjected;
        Mat copyAcquired;
        projected.copyTo(copyProjected);
        acquired.copyTo(copyAcquired);

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
        imwrite("arucoacquired.jpeg",copyAcquired);
        imwrite("arucoprojected.jpeg",copyProjected);
        //matching project pixels and cam pixels based on aruco id:
        for(int i = 0; i < cam_charucoIds.size(); i++){
            for(int j = 0; j < proj_charucoIds.size(); j++){
                if(cam_charucoIds[i] == proj_charucoIds[j]){
                    camPixels.push_back(cam_charucoCorners[i]);
                    projPixels.push_back(proj_charucoCorners[j]);
                }
            }
        }
    }
    else
    {
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
        imwrite("test.jpeg",whiteImage);
    }
    //compute extrinsic
    Mat ret (4, 4, CV_64F, Scalar (0));
    if (camPixels.size() == projPixels.size() && camPixels.size() > 9) {
        Mat F = findFundamentalMat (camPixels, projPixels, FM_RANSAC, 3, 0.99);
        //Mat E = cp_interface->getProjectorCalibration().intrinsics.t() * F * cp_interface->getCameraCalibration().intrinsics;
        Mat E = cp_interface->getCameraCalibration().intrinsics.t() * F * cp_interface->getProjectorCalibration().intrinsics;
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
        Mat R2 = U * Wt * Vt;
        Mat u1 = U.col (2);
        Mat t2 = -U.col (2);
        //4 candidates
        //save R1, u1 to "cam_proj_trans.yaml"
        saveExtrinsics(R1, u1, "../data/cam_proj_trans.yaml");

        //wrap into one single Mat to return
        ret.at<double> (3, 3) = 1;
        Mat mask_R (4, 4, CV_64F, Scalar (0));
        mask_R (Rect (0, 0, R1.rows, R1.cols)) = 1;
        R1.copyTo (ret, mask_R);
        Mat mask_t (4, 4, CV_64F, Scalar (0));
        mask_t (Rect (0, 3, u1.rows, u1.cols));
        u1.copyTo (ret, mask_t);

    }
    else
    {
        cerr << "correspodence lists size mismatched or not enough correspondences" << endl;
    }
    return ret;
}
