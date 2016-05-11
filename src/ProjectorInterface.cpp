
#include "projector_tracker/ProjectorInterface.h"
#include <QApplication>
#include <QDesktopWidget>
#include <iostream>

ProjectorInterface::ProjectorInterface()
{

}

ProjectorInterface::~ProjectorInterface()
{

}

bool ProjectorInterface::loadIntrinsics(std::string matrix_file , std::string tag)
{
    cv::FileStorage fs( matrix_file, cv::FileStorage::READ );
    if( !fs.isOpened() )
    {
      std::cout << "Failed to open Projector Calibration Data File." << std::endl;
      return false;
    }
    // Loading calibration parameters
    fs[tag] >> intrinsics;
    return true;
}

cv::Mat ProjectorInterface::getIntrinsics()
{
    return cv::Mat(intrinsics);
}

void ProjectorInterface::projectFullscreenOnScreen(const cv::Mat& target_image, int screen_number) 
{
    cv::Mat temp(target_image.cols,target_image.rows,target_image.type());
    cv::cvtColor(target_image, temp, CV_BGR2RGB);
    QImage img(temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    
    QRect screenres = QApplication::desktop()->screenGeometry(screen_number);
    image_label.move(QPoint(screenres.x(), screenres.y()));
    image_label.resize(screenres.width(), screenres.height());
    image_label.setPixmap(QPixmap::fromImage(img));
    image_label.setScaledContents(true);
    image_label.showFullScreen();
}




