
#include <projector_tracker/CameraProjectorInterface.h>
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

void test_cameraprojector_helper(cv::Mat test_image, std::shared_ptr<CameraProjectorInterface> cpi) {
    for (int i = 0; i < 100; i++) {
        std::cout << "grabbing frame" << std::endl;
        test_image = cpi->projectAndAcquire(test_image).acquired;
        std::cout << "grabbed frame" << std::endl;
    }
}

int test_cameraprojector(int argc, char **argv) {
    QApplication app(argc, argv);
    
    std::shared_ptr<CameraInterfaceBase> ci = std::make_shared<CameraInterface>();
    std::shared_ptr<ProjectorInterfaceBase> pi= std::make_shared<ProjectorInterface>();
    std::shared_ptr<CameraProjectorInterface> cpi = std::make_shared<CameraProjectorInterface>(ci, pi);
    
    cv::Mat test_image = ci->grabFrame();
    std::cout << "grabbed frame" << std::endl;
    
    std::thread t(test_cameraprojector_helper, test_image, cpi);
    
    app.exec();
}