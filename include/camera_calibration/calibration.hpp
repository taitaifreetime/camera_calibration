#pragma once

#include <string>
#include <opencv2/opencv.hpp>
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
        const double &square_size);
    ~CameraCalibration();

    bool loadImgs(
        const std::string &dir, 
        std::vector<cv::Mat> &cv_imgs) const;

    
    static bool loadParameter(
        const std::string &file_name, 
        int &width, int &height, 
        std::string &camera_name, 
        cv::Mat &camera_matrix, 
        std::string &distortion_model, 
        cv::Mat &dist_coeffs, 
        cv::Mat &rectification_matrix, 
        cv::Mat &projection_matrix);

    void calibrateMonocularCamera(
        const std::vector<cv::Mat> &imgs) const;

    void calibrateStereoCamera(
        const std::vector<cv::Mat> &left_imgs, 
        const std::vector<cv::Mat> &right_imgs) const;


private:
    bool checkRequirements(
        const std::vector<cv::Mat> &imgs) const;

    bool checkRequirements(
        const std::vector<cv::Mat> &left_imgs, 
        const std::vector<cv::Mat> &right_imgs) const;

    std::string formatParameter(
        const int &img_width, const int &img_height, 
        const cv::Mat &K, const cv::Mat &D, 
        const cv::Mat &R, const cv::Mat &P
    ) const;

    std::string formatMatrix(
        const std::string &param_type, 
        const cv::Mat &matrix
    ) const;

    const cv::Size board_size_;
    float square_size_;
    static constexpr int num_imgs_ = 10;

    
};