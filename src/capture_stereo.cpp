#include "utils/utils.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>

int main(int argc, char *argv[])
{
    int camera_id = std::atoi(argv[1]);
    int camera_id_2 = std::atoi(argv[2]);
    int width = std::atoi(argv[3]);
    int height = std::atoi(argv[4]);
    int fps = std::atoi(argv[5]);
    int flip_method = std::atoi(argv[6]);
    int flip_method_2 = std::atoi(argv[7]);
    std::string img_save_dir = argv[8];
    std::string img_save_dir_2 = argv[9];
    if (!std::filesystem::exists(img_save_dir))
    {
        std::cerr<< img_save_dir << " not found" << std::endl;
        return -1;
    }
    if (!std::filesystem::exists(img_save_dir_2))
    {
        std::cerr<< img_save_dir_2 << " not found" << std::endl;
        return -1;
    }

    cv::VideoCapture cap(utils::capture::getGstreamer(camera_id, width, height, fps, flip_method), cv::CAP_GSTREAMER);
    cv::VideoCapture cap_2(utils::capture::getGstreamer(camera_id_2, width, height, fps, flip_method_2), cv::CAP_GSTREAMER);

    if(!cap.isOpened() || !cap_2.isOpened()) {
        std::cerr << "Fail to open" << std::endl;
        return -1;
    }

    cv::Mat frame, frame_2;
    while(true) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        std::string file_name = std::to_string(ms);
        cap.read(frame);
        cap_2.read(frame_2);
        if(frame.empty() || frame_2.empty()) continue;
        cv::imshow("Image", frame);
        cv::imshow("Image 2", frame_2);
        int key = cv::waitKey(1);
        if(key == 27) break; // ESC
        else if(key == 115){ // s
            cv::imwrite(img_save_dir  +file_name+".png", frame);
            cv::imwrite(img_save_dir_2+file_name+".png", frame_2);
            std::cout << "Save " << img_save_dir  +file_name+".png" << std::endl;
            std::cout << "Save " << img_save_dir_2+file_name+".png" << std::endl;
        }
    }

    cap.release();
    cap_2.release();
    cv::destroyAllWindows();
    return 0;
}