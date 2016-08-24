
#include <projector_tracker/CameraProjectorInterface.h>
#include <projector_tracker/CameraCalibration.h>
#include <QApplication>
#include <thread>
#include <iostream>

int test_fullscreen_window(int argc, char **argv);
int test_framegrabbing(int argc, char **argv);
int test_cameraprojector(int argc, char **argv);

int main(int argc, char **argv) {
    return test_cameraprojector(argc, argv);
}

int test_fullscreen_window(int argc, char** argv) {
    QApplication app(argc, argv);
    
    ProjectorInterface pi;
    cv::Mat test_image = cv::imread("resources/test_image.jpg");
    if (! test_image.data ) {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    pi.projectFullscreen(test_image);
    
    app.exec();
}

int test_framegrabbing(int argc, char **argv) {
    QApplication app(argc, argv);
    
    CameraInterface ci;
    cv::Mat test_image = ci.grabFrame();
    
    ProjectorInterface pi;

    pi.projectFullscreen(test_image);
    
    app.exec();
}
bool calibrateCamera(cv::Mat img, std::shared_ptr<CameraCalibration> calibrationCamera){

    //CameraCalibration & calibrationCamera = camProjCalib.getCalibrationCamera();
	int numBoardsBeforeCleaning = 10;
	int numBoardsFinalCamera = 5;
	float maxReprojErrorCamera = 0.025;
    bool bFound = calibrationCamera->add(img);
    if(bFound){

        cout << "Found board!" << endl;

        calibrationCamera->calibrate();

        if(calibrationCamera->size() >= numBoardsBeforeCleaning) {

            cout << "Cleaning" << endl;

            calibrationCamera->clean(maxReprojErrorCamera);

            if(calibrationCamera->getReprojectionError(calibrationCamera->size()-1) > maxReprojErrorCamera) {
                cout << "Board found, but reproj. error is too high, skipping" << endl;
                return false;
            }
        }

        if (calibrationCamera->size()>= numBoardsFinalCamera) {

            calibrationCamera->save("calibrationCamera.yml");

            cout << "Camera calibration finished & saved to calibrationCamera.yml" << endl;

        }
    }
    else cout << "Could not find board" << endl;

    return bFound;
}
void calib_cam( std::shared_ptr<CameraInterface> ci){
    //create cameraprojector interface
    std::shared_ptr<CameraCalibration> camCalib = std::make_shared<CameraCalibration>();
    while(!camCalib->isReady()){
    	cv::Mat captured = ci->grabFrame();
    	calibrateCamera(captured,camCalib );
    }
    /*
     * //load configuration for tracking algorithm
    projTracker->loadSetting("../data/setting.yml", "use aruco pattern", "known 3D Object", "aruco width", "aruco height","aruco board number", "pattern width", "pattern height", "square size");
    std::vector<cv::Mat> patterns = projTracker->getPatternImages();
    std::vector<CameraProjectorInterface::CameraProjectorImagePair> cp_img_pairs = cpi->projectAndAcquire(patterns);
    projTracker->computeRelativePosition(cp_img_test_tracker_helperpairs);*/

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
    std::thread t(calib_cam, cam_interface);
    app.exec();
}
