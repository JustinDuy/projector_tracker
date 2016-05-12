#include <projector_tracker/ProjectorTracker.h>
#include <projector_tracker/CameraProjectorInterface.h>

#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

int main (int argc, char **argv) {
    CameraInterface cam_intrinsics;
    ProjectorInterface pro_intrinsics;
    
    pro_intrinsics.loadIntrinsics("../data/calibration.yml", "proj_K");//TO DO's: recalibration projector and camera intrinsics
    cam_intrinsics.loadIntrinsic("../data/calibration.yml", "cam_K");

    const size_t CAMERA_WIDTH = 640;//1280
    const size_t CAMERA_HEIGHT = 480;//720
    
    ProjectorTracker tracker (cam_intrinsics.get(), pro_intrinsics.get()) ;
    vector<Mat> generated_patterns = tracker.getPatternImages (CAMERA_WIDTH, CAMERA_HEIGHT);
    
    CameraProjectorInterface pro_cam_handler(cam_intrinsics.get(), CAMERA_WIDTH, CAMERA_HEIGHT, pro_intrinsics.get(), CV_CAP_OPENNI);
    vector<Mat> captured_patterns = pro_cam_handler.projectAndAcquire(generated_patterns);
    
    Mat pro_cam_extrinsic = tracker.computeRelativePosition (captured_patterns);
    // get Rotation and translation part form extrinsic
    Mat Ra = pro_cam_extrinsic(Rect (0, 0, 3, 3));
    Mat ta = pro_cam_extrinsic(Rect (0, 3, 3, 1)); //library output ....[]

    //TO DO's: example use of the library
    //read in Robot Hand-Base transformation: Rb. tb
    Mat Rb, tb;
    //read in camera-Base transformation Rz, tz
    Mat Rz, tz;
    //Compute Hand-Projector : Rx, tx
    Mat Rx = Ra.t()*(Rz*Rb);
    Mat tx = Ra.t()*(Rz*tb + tz - ta);

    return 0;
}
