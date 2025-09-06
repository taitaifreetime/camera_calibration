#include "camera_calibration/calibration.hpp"
#include <string>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    CameraCalibration cam_calib(std::stoi(argv[1]), std::stoi(argv[2]), std::stof(argv[3]));

    // monocular 
    std::vector<cv::Mat> imgs;
    std::string path = argv[4];
    cam_calib.loadImgs(path, imgs);
    cam_calib.calibrateMonocularCamera(imgs);

    return 0;
}