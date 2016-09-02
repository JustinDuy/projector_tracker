
#include <projector_tracker/CameraProjectorInterface.h>
#include <projector_tracker/ProjectorCalibration.h>
#include <QApplication>
#include <thread>
#include <iostream>

int test_fullscreen_window(int argc, char **argv);
int test_framegrabbing(int argc, char **argv);
int test_cameraprojector(int argc, char **argv);

int main(int argc, char **argv) {
    //return test_framegrabbing(argc, argv);
    //return test_fullscreen_window(argc, argv);
    return test_cameraprojector(argc, argv);
}

int test_fullscreen_window(int argc, char** argv) {
    QApplication app(argc, argv);
    
    ProjectorInterface pi;
    cv::Mat test_image = cv::imread("../data/boadaruco.jpg");
    if (! test_image.data ) {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    pi.projectFullscreen(test_image);
    
    app.exec();
}

int test_framegrabbing(int argc, char **argv) {
    QApplication app(argc, argv);
    
    //CameraInterface ci;

     int cam_deviceID = atoi(argv[1]);
    std::cout << "using camera device : " << cam_deviceID << std::endl;
    std::shared_ptr<CameraInterface> ci = std::make_shared<CameraInterface>(cam_deviceID);
    cv::Mat test_image = ci->grabFrame();
    ProjectorInterface pi;

    pi.projectFullscreen(test_image);
    
    app.exec();
}
void processImageForCircleDetection(const Mat& img, Mat& processedImg){
    Mat hsv;
    cvtColor(img,hsv,CV_BGR2HSV);
    Scalar hsv_l(100,50,50);
    Scalar hsv_h(150,255,255);

    Mat bw;
    inRange(hsv,hsv_l,hsv_h,bw);
    processedImg =  cv::Scalar::all(255) - bw;
    imwrite("circle_detect.jpeg", processedImg);
}

bool calibrateProjector(cv::Mat img, std::shared_ptr<ProjectorCalibration> calibrationProjector ){
    Mat processedImg;
    processImageForCircleDetection(img, processedImg);
    vector<cv::Point2f> corners;
    bool bFound = calibrationProjector->add(img, processedImg,corners);
    bool bCalibDone = false;
    if(bFound){
            //cout << "Found board!" << endl;
            calibrationProjector->calibrate();
            if(calibrationProjector->size() >= calibrationProjector->numBoardsBeforeCleaning) {

                    cout << "Cleaning" << endl;

                    calibrationProjector->clean();

                    if(calibrationProjector->getReprojectionError(calibrationProjector->size()-1) > calibrationProjector->maxReprojectionError) {
                            cout << "Board found, but reproj. error is too high, skipping" << endl;
                            return false;
                    }
            }

            if (calibrationProjector->size()>= calibrationProjector->numBoardsFinalCamera) {
                    if(calibrationProjector->getReprojectionError() < calibrationProjector->maxReprojectionError)
                    {
                            calibrationProjector->save("../data/calibrationProjector.yml");
                            cout << "Projector calibration finished & saved to calibrationProjector.yml" << endl;
                            bCalibDone = true;
                    }
            }
    }
    else 
        cout << "Could not find board" << endl;

    return bCalibDone;

}

void calib_proj( std::shared_ptr<CameraProjectorInterface> cpi){
    std::shared_ptr<ProjectorCalibration> projCalib = std::make_shared<ProjectorCalibration>();
    //load configuration for tracking algorithm
    projCalib->loadSetting("../data/proj_calib_setting.yml", "width", "height", "pattern type",
    		"pattern width", "pattern height", "square size", "corner x", "corner y",
    		"checker width", "checker height", "checker square",
    		"max reprojection error", "numBoardsBeforeCleaning", "numBoardsFinalCamera"
    		,"../data/calibrationCamera.yml", "cameraMatrix", "distCoeffs");
    cv::Mat test_image;
    projCalib->getPattern(test_image);
    bool finished = false;
    cv::imwrite("pattern.jpg",test_image);

    while(!finished){
        CameraProjectorInterface::CameraProjectorImagePair img_pair= cpi->projectAndAcquire(test_image);
        Mat captured = img_pair.acquired;
        finished = calibrateProjector(captured, projCalib );
        //display for debug
        cv::namedWindow( "captured", cv:: WINDOW_NORMAL );// Create a window for display.
        resizeWindow( "captured", 640, 480 );
        // Moving window of captured to see the image at first screen
        moveWindow( "captured", 640, 0 );
	imshow( "captured", captured );                   // Show our image inside it.
	waitKey(5);
    }
   

}
void test_cameraprojector_helper( std::shared_ptr<CameraProjectorInterface> cpi) {
    std::vector<cv::Mat> test_images;
    cv::Mat test = cv::imread("../data/boadaruco.jpg");
    for(int i = 0; i < 100; i++){
        test_images.push_back(test);
    }
    std::vector<CameraProjectorInterface::CameraProjectorImagePair> cp_img_pairs = cpi->projectAndAcquire(test_images);
}

int test_cameraprojector(int argc, char **argv) {
    QApplication app(argc, argv);
    //read second arg for camera device id
    int cam_deviceID = atoi(argv[1]);
    std::cout << "using camera device : " << cam_deviceID << std::endl;
    std::shared_ptr<CameraInterface> cam_interface = std::make_shared<CameraInterface>(cam_deviceID);
    std::shared_ptr<ProjectorInterface> proj_interface= std::make_shared<ProjectorInterface>();
	std::shared_ptr<CameraProjectorInterface> cpi = std::make_shared<CameraProjectorInterface>(cam_interface, proj_interface, 500);
	std::thread t(calib_proj, cpi);
    app.exec();
}
