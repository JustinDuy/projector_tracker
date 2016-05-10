
#include <projector_tracker/CameraProjectorInterface.h>
#include <string>
#include <iostream>

CameraProjectorInterface::CameraProjectorInterface(const cv::Mat& camera_intrinsics, size_t w, size_t h, const cv::Mat& projector_intrinsics, int webcam_device)
: camera_intrinsics(camera_intrinsics)
, projector_intrinsics(projector_intrinsics)
, camera_width(w), camera_height(h)
{
    capture = cv::VideoCapture(webcam_device);//
    if( !capture.isOpened() )
    {
      // check if cam opened
      std::cout << "Error: camera device is not opened!" << std::endl;
    }
}

CameraProjectorInterface::~CameraProjectorInterface()
{
    // TODO: release camera and projector if necessary
    if(capture.isOpened()){
        capture.release();
    }
}


cv::Mat CameraProjectorInterface::projectAndAcquire(const cv::Mat& target_image, int delay_ms, int seq)
{
    // TODO: send the target_image to the projector, wait for it to project it, grab through camera
    std::string captured_path = "../captured/";
    cv::imshow( "Pattern Window", target_image);
    cv::Mat frame1;
    do{
        capture.grab();
        capture.retrieve( frame1, CV_CAP_OPENNI_BGR_IMAGE );

        if(frame1.data )
        {
            cv::Mat gray;
            // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
            cv::resize( frame1, gray, cv::Size( camera_width, camera_height ) );
            cv::cvtColor( gray, gray, cv::COLOR_RGB2GRAY );
            bool save1 = false;

            std::ostringstream name;
            name << seq ;
            save1 = cv::imwrite( captured_path + "im" + name.str() + ".png", frame1 );
            if( save1 )
            {
                std::cout << "pattern camera image number " << seq << " saved" << std::endl ;
            }
            else
            {
                std::cout << "pattern cam images number " << seq<< " NOT saved" << std::endl << "Retry, check the path"<< std::endl;
            }

        }
        else
        {
            std::cout << "No frame data, waiting for new frame" << std::endl;
        }
    }
    while(!frame1.data);
}

std::vector<cv::Mat> CameraProjectorInterface::projectAndAcquire(const std::vector<cv::Mat>& target_images)
{
    // Setting pattern window on second monitor (the projector's one)
    cv::namedWindow( "Pattern Window", cv::WINDOW_NORMAL );
    cv::resizeWindow( "Pattern Window", camera_width*2, camera_height*2 );
    cv::moveWindow( "Pattern Window", camera_width + 1000, 50 );
    std::vector<cv::Mat> ret;
    int i =0;
    for (auto& target_image: target_images){
        ret.push_back(projectAndAcquire(target_image, i == 0 ? 500 : 250 , i+1));
        i++;
    }
    return ret;
}
