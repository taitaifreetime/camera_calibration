#include "camera_calibration/calibration.hpp"
#include <string>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    CameraCalibration cam_calib(std::stoi(argv[1]), std::stoi(argv[2]), std::stof(argv[3]));

    // stereo
    std::vector<cv::Mat> left_imgs;
    std::string left_path = argv[4];
    cam_calib.loadImgs(left_path, left_imgs);
    std::vector<cv::Mat> right_imgs;
    std::string right_path = argv[5];
    cam_calib.loadImgs(right_path, right_imgs);
    cam_calib.calibrateStereoCamera(left_imgs, right_imgs);

    return 0;
}