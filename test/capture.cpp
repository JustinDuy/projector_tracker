
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
bool calib_captured( std::shared_ptr<CameraInterface> ci){
    int count = 0;
    char buff[250];
    while(true){
        cv::Mat captured = ci->grabFrame();
        sprintf(buff,"captured_%d.jpg", count);
        imwrite(buff, captured); 
        cout << buff << " is saved"<< endl;
        count++;
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
    std::thread t(calib_captured, cam_interface);

    app.exec();
}
