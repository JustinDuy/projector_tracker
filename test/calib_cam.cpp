#include <dirent.h>
#include <projector_tracker/CameraProjectorInterface.h>
#include <projector_tracker/CameraCalibration.h>
#include <QApplication>
#include <thread>
#include <iostream>
using namespace cv;
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
bool calibrateCamera(std::vector<cv::Mat> imgs, std::shared_ptr<CameraCalibration> calibrationCamera){
    bool bCalibDone = false;
    for(int i = 0; i < imgs.size() ; i++){
        cv::Mat img = imgs[i];
        vector<cv::Point2f> corners;
        bool bFound = calibrationCamera->add(img,corners);
        if(bFound){
            calibrationCamera->drawCheckerBoard(img,corners);
            cout << "Found board on image " << i << endl;
        }
    }
    cout << "calibrating ..." << endl;
    calibrationCamera->calibrate();
    if(calibrationCamera->getReprojectionError() < calibrationCamera->maxReprojectionError){
        calibrationCamera->save("../data/calibrationCamera.yml");        	
        cout << "Camera calibration finished & saved to calibrationCamera.yml" << endl;
        bCalibDone = true;
    }
    else 
    {
        cout << "Camera calibration reprojection error is too high " << calibrationCamera->getReprojectionError()<< endl;
        
    }
    return bCalibDone;
}
bool calibrateCamera(cv::Mat img, std::shared_ptr<CameraCalibration> calibrationCamera){
   // if(calibrationCamera->updateCamDiff(img)){
	vector<cv::Point2f> corners;
        bool bFound = calibrationCamera->add(img,corners);
        bool bCalibDone = false;
        if(bFound){
            calibrationCamera->drawCheckerBoard(img,corners);
            cout << "Found board!" << endl;

            calibrationCamera->calibrate();

            if(calibrationCamera->size() >= calibrationCamera->numBoardsBeforeCleaning) {

                cout << "Cleaning" << endl;

                calibrationCamera->clean();

                if(calibrationCamera->getReprojectionError(calibrationCamera->size()-1) > calibrationCamera->maxReprojectionError) {
                    cout << "Board found, but reproj. error is too high, skipping" << endl;
                    return false;
                }
            }

            if (calibrationCamera->size()>= calibrationCamera->numBoardsFinalCamera) {
                    if(calibrationCamera->getReprojectionError() < calibrationCamera->maxReprojectionError)
                    {
                            calibrationCamera->save("../data/calibrationCamera.yml");        	
                            cout << "Camera calibration finished & saved to calibrationCamera.yml" << endl;
                            bCalibDone = true;
                    }
            }
        //}
        //else cout << "Could not find board" << endl;

        return bCalibDone;
        
    }
    else
    {
        cout << "move the board" << endl;
        return false;
    }
}
void calib_cam( std::shared_ptr<CameraInterface> ci){
    //create cameraprojector interface
    std::shared_ptr<CameraCalibration> camCalib = std::make_shared<CameraCalibration>();
    //load configuration for tracking algorithm
    camCalib->loadSetting("../data/cam_calib_setting.yml", "pattern type", 
    		"pattern width", "pattern height", "square size", "max reprojection error", "numBoardsBeforeCleaning", "numBoardsFinalCamera");
	bool finished = false;
    while(!finished){
    	cv::Mat captured = ci->grabFrame();
    	finished = calibrateCamera(captured,camCalib );
    }
}
void calib_cam_img( std::string calibImgPath){
    std::shared_ptr<CameraCalibration> camCalib = std::make_shared<CameraCalibration>();
    //load configuration for tracking algorithm
    camCalib->loadSetting("../data/cam_calib_setting.yml", "pattern type", 
    		"pattern width", "pattern height", "square size", "max reprojection error", "numBoardsBeforeCleaning", "numBoardsFinalCamera");
    DIR *dpdf;
    struct dirent *epdf;

    dpdf = opendir(calibImgPath.c_str());
    char fullPathFile[200];
    if (dpdf != NULL){
        std::vector<cv::Mat> imgs;
        while (epdf = readdir(dpdf)){
            printf("Filename: %s\n",epdf->d_name);
            sprintf(fullPathFile, "%s\/%s",  calibImgPath.c_str(),epdf->d_name);
            //std::cout << "loading " << epdf->d_name << std::endl;
            Mat image;
            
            image = imread(fullPathFile, CV_LOAD_IMAGE_COLOR);   // Read the file
            if(! image.data )                              // Check for invalid input
            {
                cout <<  "Could not open or find the image " << fullPathFile << std::endl ;
                continue;
            }
            else{
                cv::Mat gray;
                cv::cvtColor(image, gray, CV_BGR2GRAY);
                imgs.push_back(gray);
            }
        }
        calibrateCamera(imgs, camCalib);
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
    /*int cam_deviceID = atoi(argv[1]);
    std::cout << "using camera device : " << cam_deviceID << std::endl;
    std::shared_ptr<CameraInterface> cam_interface = std::make_shared<CameraInterface>(cam_deviceID);
    std::thread t(calib_cam, cam_interface);*/
    calib_cam_img(argv[1]);
    app.exec();
}
