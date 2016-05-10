#include <projector_tracker/ProjectorTracker.h>
#include <projector_tracker/CameraProjectorInterface.h>
#include <vector>
#include <iostream>
using namespace cv;
using namespace std;
int main (int argc, char **argv) {
    //load Kinect calibration file
    FileStorage fs ("../data/rgb_A00363813595051A.yaml", FileStorage::READ);

    if (!fs.isOpened()) {
        cout << "Failed to open Camera Calibration Data File." << endl;
        return -1;
    }

    FileStorage fs2 ("../data/calibration.yml", FileStorage::READ);

    if (!fs2.isOpened()) {
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

    if ( (!cam1intrinsics.data) || (!cam1distCoeffs.data) || (!prointrinsics.data)) {
        cout << "Failed to load camera/projector calibration parameters" << endl;
        return -1;
    }

    const size_t CAMERA_WIDTH = 640;//1280
    const size_t CAMERA_HEIGHT = 480;//720

    ProjectorTracker tracker (cam1intrinsics, prointrinsics) ;
    vector<Mat> generated_patterns = tracker.getPattern (CAMERA_WIDTH, CAMERA_HEIGHT);
    CameraProjectorInterface pro_cam_handler(cam1intrinsics, CAMERA_WIDTH, CAMERA_HEIGHT, cam1intrinsics, CV_CAP_OPENNI);
    vector<Mat> captured_patterns = pro_cam_handler.projectAndAcquire(generated_patterns);
    //camera-base is known and assume scale factor is known
    //estimate Ra, ta: using graycode
    Mat pro_cam_extrinsic = tracker.computeRelativePosition (captured_patterns);

    return 0;
}
