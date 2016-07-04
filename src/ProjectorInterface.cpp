
#include "projector_tracker/ProjectorInterface.h"
#include <QApplication>
#include <QDesktopWidget>

#include <iostream>
#include <thread>


QImage Mat2QImage(const cv::Mat& inImg);
QImage Gray2QImage(const cv::Mat& inImg);
QImage Rgb2QImage(const cv::Mat& inImg);

ProjectorInterface::ProjectorInterface()
{

}

ProjectorInterface::~ProjectorInterface()
{

}

bool ProjectorInterface::loadIntrinsics(std::string matrix_file , std::string tag_K, std::string tag_W, std::string tag_H)
{
    cv::FileStorage fs( matrix_file, cv::FileStorage::READ );
    if( !fs.isOpened() )
    {
      std::cout << "Failed to open Projector Calibration Data File." << std::endl;
      return false;
    }
    // Loading calibration parameters
    fs[tag_K] >> calibration.intrinsics;
    int width, height;
    fs[tag_W] >> width;
    fs[tag_H] >> height;
    calibration.width = width;
    calibration.height = height;
    return true;
}

ProjectorInterfaceBase::Calibration ProjectorInterface::getCalibration()
{
    return calibration;
}


void ProjectorInterface::projectFullscreenOnScreen(const cv::Mat& target_image, int screen_number) 
{
    QImage  qImg = Mat2QImage(target_image);
    QRect screenres = QApplication::desktop()->screenGeometry(screen_number);
    image_label.move(QPoint(screenres.x(), screenres.y()));
    image_label.resize(screenres.width(), screenres.height());
    QPixmap qpx = QPixmap::fromImage(qImg,Qt::MonoOnly);
    image_label.setPixmap(qpx);
    //image_label.setScaledContents(true);
    image_label.showFullScreen();
}
QImage Mat2QImage(const cv::Mat& inImg){
    int type = inImg.type();
    if(type == CV_8UC1){
        return Gray2QImage(inImg);
    }
    else if(type == CV_8UC3){
        return Rgb2QImage(inImg);
    }
    else {
        std::cerr << "GrayCode Format unsupported!" << std::endl;
        return QImage();
    }
}


QImage Gray2QImage(const cv::Mat& inMat)
{
    static QVector<QRgb>  sColorTable;
    // only create our color table once
    if ( sColorTable.isEmpty() )
    {
       for ( int i = 0; i < 256; ++i )
          sColorTable.push_back( qRgb( i, i, i ) );
    }
    QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );
    image.setColorTable( sColorTable );
    return image;
}

QImage Rgb2QImage(const cv::Mat& inMat) {
    QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );
    return image.rgbSwapped();
}
