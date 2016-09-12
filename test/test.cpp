
#include <projector_tracker/CameraProjectorInterface.h>
#include <projector_tracker/ProjectorTracker.h>
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
bool calib_proj( std::shared_ptr<CameraProjectorInterface> cpi){
    //load configuration for tracking algorithm
    std::shared_ptr<ProjectorTracker> projTracker = std::make_shared<ProjectorTracker>  ();
    //load configuration for tracking algorithm
    projTracker->loadSetting("../data/setting.yml", "use aruco pattern", "known 3D Object", "aruco width", "aruco height","aruco board number", "pattern width", "pattern height", "square size", 
                             "max intrinsic reprojection error", "max extrinsic reprojection error", "numBoardsBeforeCleaning", "numBoardsFinalCamera",
        "../data/calibrationCamera.yml", "cameraMatrix", "distCoeffs", "imageSize_width", "imageSize_height", "../data/calibrationProjector.yml", "projectorMatrix", "distCoeffs", "projectorWidth", "projectorHeight"
    );
    vector<cv::Mat> test_images = projTracker->getPatternImages();
    cv::imwrite("pattern.jpg",test_images[0]);
    Mat test_image = test_images[0];
    bool finished = false;
    while(!finished){
        CameraProjectorInterface::CameraProjectorImagePair img_pair= cpi->projectAndAcquire(test_image);
        Mat captured = img_pair.acquired;
        finished = projTracker->run(test_image, captured);
        
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
	std::shared_ptr<CameraProjectorInterface> cpi = std::make_shared<CameraProjectorInterface>(cam_interface, proj_interface, 200);
	std::thread t(calib_proj, cpi);
    app.exec();
}
