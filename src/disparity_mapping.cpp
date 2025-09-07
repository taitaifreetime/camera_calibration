#include "camera_calibration/calibration.hpp"
#include "utils/computer_vision.hpp"

int main(int argc, char* argv[])
{
    std::string left_camera_parameter_file = argv[1];
    std::string right_camera_parameter_file = argv[2];
    int left_camera_id = std::atoi(argv[3]);
    int right_camera_id = std::atoi(argv[4]);
    int fps = std::atoi(argv[5]);
    int left_flip_method = std::atoi(argv[6]);
    int right_flip_method = std::atoi(argv[7]);

    int left_width=0, left_height=0; 
    std::string left_camera_name; 
    cv::Mat left_camera_matrix;
    std::string left_distortion_model; 
    cv::Mat left_dist_coeffs;
    cv::Mat left_rectification_matrix; 
    cv::Mat left_projection_matrix;
    CameraCalibration::loadParameter(
        left_camera_parameter_file, 
        left_width, left_height, 
        left_camera_name, 
        left_camera_matrix, 
        left_distortion_model, 
        left_dist_coeffs, 
        left_rectification_matrix, 
        left_projection_matrix
    );
    std::cout << "========== Left Camera Parameters ==========" << std::endl;
    std::cout << "Image width:  " << left_width << std::endl;
    std::cout << "Image height: " << left_height << std::endl;
    std::cout << "Camera name:  " << left_camera_name << std::endl;
    std::cout << "Camera Matrix:\n" << left_camera_matrix << std::endl;
    std::cout << "Distortion Model: " << left_distortion_model << std::endl;
    std::cout << "Distortion Coeffs:\n" << left_dist_coeffs << std::endl;
    std::cout << "Rectification Matrix:\n" << left_rectification_matrix << std::endl;
    std::cout << "Projection Matrix:\n" << left_projection_matrix << std::endl;
    std::cout << "============================================" << std::endl;


    int right_width=0, right_height=0; 
    std::string right_camera_name; 
    cv::Mat right_camera_matrix;
    std::string right_distortion_model; 
    cv::Mat right_dist_coeffs;
    cv::Mat right_rectification_matrix; 
    cv::Mat right_projection_matrix;
    CameraCalibration::loadParameter(
        right_camera_parameter_file, 
        right_width, right_height, 
        right_camera_name, 
        right_camera_matrix, 
        right_distortion_model, 
        right_dist_coeffs, 
        right_rectification_matrix, 
        right_projection_matrix
    );
    std::cout << "========== Right Camera Parameters ==========" << std::endl;
    std::cout << "Image width:  " << right_width << std::endl;
    std::cout << "Image height: " << right_height << std::endl;
    std::cout << "Camera name:  " << right_camera_name << std::endl;
    std::cout << "Camera Matrix:\n" << right_camera_matrix << std::endl;
    std::cout << "Distortion Model: " << right_distortion_model << std::endl;
    std::cout << "Distortion Coeffs:\n" << right_dist_coeffs << std::endl;
    std::cout << "Rectification Matrix:\n" << right_rectification_matrix << std::endl;
    std::cout << "Projection Matrix:\n" << right_projection_matrix << std::endl;
    std::cout << "============================================" << std::endl;

    CameraCalibration calib(0, 0, 0.0);
    calib.setParameter(
        left_width, left_height, 
        left_camera_matrix, 
        left_dist_coeffs, 
        left_rectification_matrix, 
        left_projection_matrix, 
        right_width, right_height, 
        right_camera_matrix, 
        right_dist_coeffs, 
        right_rectification_matrix, 
        right_projection_matrix
    );
    calib.makeRectifiedMap();

    // StereoSGBM parameters
    int numDisparities = 16*3;
    int blockSize = 5; // lower -> faster
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0,    // minDisparity: Minimum disparity value.
        numDisparities, // numDisparities: Number of disparities (must be divisible by 16).
        blockSize, // blockSize: Size of the SAD window.
        8 * 1 * blockSize * blockSize,  // P1: First disparity smoothness parameter.
        32 * 1 * blockSize * blockSize, // P2: Second disparity smoothness parameter.
        1,    // disp12MaxDiff: Maximum allowed difference in the left-right disparity check.
        31,   // preFilterCap: Truncation value for prefiltered image pixels.
        10,   // uniquenessRatio: Margin by which the best cost should win the second best.
        50,   // speckleWindowSize: Maximum size of smooth disparity regions to consider as noise.
        2,    // speckleRange: Maximum disparity variation within each connected component.
        cv::StereoSGBM::MODE_SGBM_3WAY // mode: SGBM algorithm mode (e.g., MODE_SGBM, MODE_HH). MODE_SGBM_3WAY for arm core.
    );
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbm);
    cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(sgbm);

    cv::VideoCapture capL(ComputerVision::getGstreamer(left_camera_id, left_width, left_height, fps, left_flip_method), cv::CAP_GSTREAMER);
    cv::VideoCapture capR(ComputerVision::getGstreamer(right_camera_id, right_width, right_height, fps, right_flip_method), cv::CAP_GSTREAMER);
    if(!capL.isOpened() || !capR.isOpened()) {
        std::cerr << "Fail to open" << std::endl;
        return -1;
    }

    cv::Mat frameL, frameR, disparity;
    float depth_scale = 255.0/(numDisparities*16.0);
    while(true) {
        #ifdef DEBUG
        auto start = std::chrono::steady_clock::now();
        #endif
        capL.read(frameL);
        capR.read(frameR);
        if(frameL.empty() || frameR.empty()) continue;

        // calc depth image
        // calib.calcDepthImage(frameL, frameR, disparity, sgbm, depth_scale);
        calib.calcDepthImageWithPostProc(
            frameL, frameR, disparity, sgbm, depth_scale, 
            right_matcher, wls_filter, 8000.0, 3.5);
        

        #ifdef DEBUG
        auto end = std::chrono::steady_clock::now();
        auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << 1000.0/time_ms << "FPS" << std::endl;
        cv::imshow("Left", frameL);
        cv::imshow("Right", frameR);
        #endif
        cv::imshow("Disparity", disparity);

        if(cv::waitKey(1) == 27) break; // ESC
    }

    capL.release();
    capR.release();
    cv::destroyAllWindows();
    return 0;
}