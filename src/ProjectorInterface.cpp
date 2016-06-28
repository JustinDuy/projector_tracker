
#include "projector_tracker/ProjectorInterface.h"
#include <QApplication>
#include <QDesktopWidget>
#include <iostream>
QImage  cvMatToQImage( const cv::Mat &inMat );
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
    fs[tag] >> calibration.intrinsics;
    int width, height;
    fs["imageSize_width"] >> width;
    fs["imageSize_height"] >> height;
    calibration.width = width;
    calibration.height = height;
    //std::cout << "projector resolution : " << width << " , " << height << std::endl;
    return true;
}

ProjectorInterfaceBase::Calibration ProjectorInterface::getCalibration()
{
    return calibration;
}


void ProjectorInterface::projectFullscreenOnScreen(const cv::Mat& target_image, int screen_number) 
{
    QImage  qImg = cvMatToQImage( target_image );
    QRect screenres = QApplication::desktop()->screenGeometry(screen_number);
    image_label.move(QPoint(screenres.x(), screenres.y()));
    image_label.resize(screenres.width(), screenres.height());
    image_label.setPixmap(QPixmap::fromImage(qImg));
    image_label.setScaledContents(true);
    image_label.showFullScreen();
}

QImage  cvMatToQImage( const cv::Mat &inMat )
{
  switch ( inMat.type() )
  {
     // 8-bit, 4 channel
     case CV_8UC4:
     {
        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32 );

        return image;
     }

     // 8-bit, 3 channel
     case CV_8UC3:
     {
        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );

        return image.rgbSwapped();
     }

     // 8-bit, 1 channel
     case CV_8UC1:
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

     default:
        std::cerr << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type() << std::endl;
        break;
  }

  return QImage();
}


