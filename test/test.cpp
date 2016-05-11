
#include <projector_tracker/ProjectorInterface.h>
#include <QApplication>
#include <iostream>

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    
    ProjectorInterface pi;
    cv::Mat test_image = cv::imread("resources/test_image.jpg");
    if (! test_image.data ) {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    pi.projectFullscreen(test_image);
    
    
    cv::Mat temp(test_image.cols,test_image.rows,test_image.type());
    cv::cvtColor(test_image, temp, CV_BGR2RGB);
    QImage img((uchar*)temp.data, temp.cols, temp.rows, temp.step1(), QImage::Format_RGB32);
     
    std::cout <<  "image size"  << img.size().width() << img.size().height()  << std::endl ;
    
    QLabel image_label;
    image_label.setPixmap(QPixmap::fromImage(img));
    image_label.setScaledContents(true);
    image_label.show();
    
    app.exec();
}