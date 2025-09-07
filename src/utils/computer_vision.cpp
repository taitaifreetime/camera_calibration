#include "utils/computer_vision.hpp"

/**
 * @brief load images from dir
 * 
 * @param dir 
 * @param cv_imgs 
 * @return true 
 * @return false if dir not found
 */
bool ComputerVision::loadImgs(
    const std::string &dir, 
    std::vector<cv::Mat> &cv_imgs
){
    if (!std::filesystem::exists(dir))
    {
        std::cerr<< dir << " not found" << std::endl;
        return false;
    }

    std::cout << "Images found in " << dir << " below" << std::endl;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) 
    {
        if (entry.is_regular_file()) 
        {
            std::string ext = entry.path().extension().string();
            if (ext == ".jpg" || ext == ".png" || ext == ".jpeg" || ext == ".bmp") 
            {
                cv::Mat img = cv::imread(entry.path().string());
                cv_imgs.push_back(img);
                std::cout << entry.path().filename().string() << std::endl;
            }
        }
    }

    return true;
}

std::string ComputerVision::getGstreamer(const int cam_id, const int width, const int height, const int freq, const int flip_method)
{
    return "nvarguscamerasrc sensor-id=" + std::to_string(cam_id) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
        std::to_string(height) + ", framerate=(fraction)" + std::to_string(freq) +
        "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(width) + ", height=(int)" +
        std::to_string(height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}