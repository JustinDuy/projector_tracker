#include <projector_tracker/ProjectorTracker.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
//#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
//#include <sensor_msgs/PointCloud2.h>
//#include "rgbprocess.h"

#include <iostream>
using namespace cv;
using namespace std;
void capturePatterns(VideoCapture& capture, const vector<Mat>& pattern, vector<Mat>& captured_patterns);
int main(int argc, char **argv) {
    //load Kinect calibration file
    FileStorage fs( "../data/rgb_A00363813595051A.yaml", FileStorage::READ );
    if( !fs.isOpened() )
    {
      cout << "Failed to open Camera Calibration Data File." << endl;
      return -1;
    }
    FileStorage fs2( "../data/calibration.yml", FileStorage::READ);
    if( !fs2.isOpened()){
        cout << "Failed to open Projector Calibration Data File." << endl;
    }

    // Loading calibration parameters
    Mat cam1intrinsics, cam1distCoeffs;
    Mat prointrinsics;

    fs["camera_matrix"] >> cam1intrinsics;
    fs["distortion_coefficients"] >> cam1distCoeffs;
    cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
    cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;

    fs2["proj_K"] >> prointrinsics;
    cout << "prointrinsics" << endl << prointrinsics << endl;
    if((!cam1intrinsics.data) || (!cam1distCoeffs.data) || (!prointrinsics.data))
    {
      cout << "Failed to load camera/projector calibration parameters" << endl;
      return -1;
    }

    const size_t CAMERA_WIDTH = 640;//1280
    const size_t CAMERA_HEIGHT = 480;//720

    ProjectorTracker tracker(cam1intrinsics, prointrinsics) ;
    vector<Mat> generated_patterns = tracker.getPattern(CAMERA_WIDTH, CAMERA_HEIGHT);

    //namedWindow( "cam1", WINDOW_NORMAL );
    //resizeWindow( "cam1", 640, 480 );

    // Setting pattern window on second monitor (the projector's one)
    namedWindow( "Pattern Window", WINDOW_NORMAL );
    resizeWindow( "Pattern Window", CAMERA_WIDTH*2, CAMERA_HEIGHT*2 );
    moveWindow( "Pattern Window", CAMERA_WIDTH + 1000, 50 );
    //setWindowProperty( "Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN );
#ifndef ___DEBUG

    //namedWindow( "cam1", WINDOW_NORMAL );
    //resizeWindow( "cam1", 640, 480 );
    // Open kinect rgb cam, using openni
    VideoCapture capture( CV_CAP_OPENNI );
    if( !capture.isOpened() )
    {
      // check if cam opened
      cout << "kinect not opened!" << endl;
      return -1;
    }

#endif

    //camera-base is known and assume scale factor is known
    //estimate Ra, ta: using graycode
    Mat Ra, ta;
    vector<Mat> captured_patterns;


    //call synchronizer to project and capture graycode patterns
    vector<Mat> capture_patterns;
    capturePatterns(capture,generated_patterns, capture_patterns);


    tracker.computeRelativePosition(captured_patterns);
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
void capturePatterns(VideoCapture& capture, const vector<Mat>& pattern, vector<Mat>& captured_patterns)
{
    String captured_path = "../captured/";
    int i =0;
    while( i < pattern.size() )
    {
      cout << "Waiting to save image number " << i + 1 << endl ;
      imshow( "Pattern Window", pattern[i] );
      if(i==0)
          waitKey(500);
      else
          waitKey(250);
      Mat frame1;
      capture.grab();
      capture.retrieve( frame1, CV_CAP_OPENNI_BGR_IMAGE );

      if(frame1.data )
      {
        Mat gray;

        // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
        resize( frame1, gray, Size( 640, 480 ) );
        cvtColor( gray, gray, COLOR_RGB2GRAY );
        bool save1 = false;

        ostringstream name;
        name << i + 1;
        save1 = imwrite( captured_path + "im" + name.str() + ".png", frame1 );
        if( save1 )
        {
            cout << "pattern camera image number " << i + 1 << " saved" << endl ;
        }
        else
        {
            cout << "pattern cam1 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path"<< endl << endl;
        }
        i++;
      }
      else
      {
        cout << "No frame data, waiting for new frame" << endl;
      }
    }
    //end while loop for capturing patterns

}
