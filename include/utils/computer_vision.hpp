#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>

class ComputerVision
{
    public:
        ComputerVision();
        ~ComputerVision();

        static bool loadImgs(
            const std::string &dir, 
            std::vector<cv::Mat> &cv_imgs
        );

        static std::string getGstreamer(
            const int cam_id, 
            const int width, 
            const int height, 
            const int freq, 
            const int flip_method
        );
        
};