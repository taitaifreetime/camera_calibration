#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/ximgproc/disparity_filter.hpp" 
#include <filesystem>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

class CameraCalibration
{
public:
    CameraCalibration(
        const int &checker_width, 
        const int &checker_height, 
        const double &square_size
    );
    ~CameraCalibration();

    static bool loadParameter(
        const std::string &file_name, 
        int &width, int &height, 
        std::string &camera_name, 
        cv::Mat &camera_matrix, 
        std::string &distortion_model, 
        cv::Mat &dist_coeffs, 
        cv::Mat &rectification_matrix, 
        cv::Mat &projection_matrix
    );

    void setParameter( 
        const int &width, const int &height, 
        const cv::Mat &camera_matrix, 
        const cv::Mat &dist_coeffs, 
        const cv::Mat &rectification_matrix, 
        const cv::Mat &projection_matrix
    );

    void setParameter( 
        const int &width1, const int &height1, 
        const cv::Mat &camera_matrix1, 
        const cv::Mat &dist_coeffs1, 
        const cv::Mat &rectification_matrix1, 
        const cv::Mat &projection_matrix1, 
        const int &width2, const int &height2, 
        const cv::Mat &camera_matrix2, 
        const cv::Mat &dist_coeffs2, 
        const cv::Mat &rectification_matrix2, 
        const cv::Mat &projection_matrix2
    );

    void setRectifiedMap(
        const cv::Mat &mapx, 
        const cv::Mat &mapy
    );

    void setRectifiedMap(
        const cv::Mat &map1x, 
        const cv::Mat &map1y,
        const cv::Mat &map2x, 
        const cv::Mat &map2y
    );

    bool makeRectifiedMap();

    void calcRectifiedImage(
        const cv::Mat &original_img, 
        cv::Mat &rect_img, 
        const int &approx = cv::INTER_NEAREST
    ) const;

    void calcRectifiedImage(
        const cv::Mat &original_img1, 
        cv::Mat &rect_img1, 
        const cv::Mat &original_img2, 
        cv::Mat &rect_img2, 
        const int &approx = cv::INTER_NEAREST
    ) const;

    void calcDepthImage(
        const cv::Mat &original_left_img, 
        const cv::Mat &original_right_img, 
        cv::Mat &depth_img, 
        const cv::Ptr<cv::StereoSGBM> &sgbm, 
        const double &depth_scale
    ) const;

    void calcDepthImageWithPostProc(
        const cv::Mat &original_left_img, 
        const cv::Mat &original_right_img, 
        cv::Mat &depth_img, 
        const cv::Ptr<cv::StereoSGBM> &left_matcher,
        const double &depth_scale, 
        const cv::Ptr<cv::StereoMatcher> &right_matcher, 
        const cv::Ptr<cv::ximgproc::DisparityWLSFilter> &wls_filter, 
        const float &lambda, 
        const float &sigma
    ) const;

    void calibrateMonocularCamera(
        const std::vector<cv::Mat> &imgs
    ) const;

    void calibrateStereoCamera(
        const std::vector<cv::Mat> &imgs1, 
        const std::vector<cv::Mat> &imgs2
    ) const;



private:
    bool checkCalibrationRequirements(
        const std::vector<cv::Mat> &imgs
    ) const;

    bool checkCalibrationRequirements(
        const std::vector<cv::Mat> &imgs1, 
        const std::vector<cv::Mat> &imgs2
    ) const;

    std::string formatParameter(
        const int &img_width, const int &img_height, 
        const cv::Mat &K, const cv::Mat &D, 
        const cv::Mat &R, const cv::Mat &P
    ) const;

    std::string formatMatrix(
        const std::string &param_name, 
        const cv::Mat &matrix
    ) const;

    const cv::Size board_size_;
    float square_size_;
    static constexpr int num_imgs_ = 10;

    int width1_, width2_;
    int height1_, height2_;
    cv::Mat camera_matrix1_, camera_matrix2_;
    cv::Mat dist_coeffs1_, dist_coeffs2_; 
    cv::Mat rectification_matrix1_, rectification_matrix2_; 
    cv::Mat projection_matrix1_, projection_matrix2_;

    cv::Mat map1x_, map2x_;
    cv::Mat map1y_, map2y_;
    
};