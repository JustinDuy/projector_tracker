
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

void test_cameraprojector_helper(std::vector<cv::Mat> test_images, std::shared_ptr<CameraProjectorInterface> cpi, std::shared_ptr<ProjectorTracker> tracker) {
    std::vector<CameraProjectorInterface::CameraProjectorImagePair> cp_img_pairs = cpi->projectAndAcquire(test_images);
    tracker->computeRelativePosition(cp_img_pairs);
}

int test_cameraprojector(int argc, char **argv) {
    QApplication app(argc, argv);
    //read second arg for camera device id
    int cam_deviceID = atoi(argv[1]);
    std::cout << "using camera device : " << cam_deviceID << std::endl;
    std::shared_ptr<CameraInterface> cam_interface = std::make_shared<CameraInterface>(cam_deviceID);
    cam_interface->loadIntrinsics("../data/rgb_A00363813595051A.yaml", "camera_matrix", "distortion_coefficients", "image_width", "image_height"); //Kinect RGB Cam
    cam_interface->loadIntrinsics("../data/calibrationCamera.yml", "cameraMatrix", "distCoeffs", "imageSize_width", "imageSize_height"); //C92 HD Webcam

    std::shared_ptr<ProjectorInterface> proj_interface= std::make_shared<ProjectorInterface>();
    proj_interface->loadIntrinsics("../data/calibrationProjector.yml", "cameraMatrix", "distCoeffs", "imageSize_width", "imageSize_height");

    std::shared_ptr<CameraProjectorInterface> cpi = std::make_shared<CameraProjectorInterface>(cam_interface, proj_interface, 400);
    std::shared_ptr<ProjectorTracker> projTracker = std::make_shared<ProjectorTracker>  (cpi);

    projTracker->loadSetting("../data/setting.yml", "use aruco pattern", "known 3D Object", "aruco width", "aruco height", "pattern width", "pattern height", "square size");
    std::vector<cv::Mat> patterns = projTracker->getPatternImages(proj_interface->getCalibration().width,
                                                                  proj_interface->getCalibration().height);
    std::thread t(test_cameraprojector_helper, patterns, cpi, projTracker);
    app.exec();
}
