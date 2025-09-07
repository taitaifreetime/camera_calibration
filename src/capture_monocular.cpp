#include "utils/utils.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>

int main(int argc, char *argv[])
{
    int camera_id = std::atoi(argv[1]);
    int width = std::atoi(argv[2]);
    int height = std::atoi(argv[3]);
    int fps = std::atoi(argv[4]);
    int flip_method = std::atoi(argv[5]);
    std::string img_save_dir = argv[6];
    if (!std::filesystem::exists(img_save_dir))
    {
        std::cerr<< img_save_dir << " not found" << std::endl;
        return -1;
    }

    cv::VideoCapture cap(utils::capture::getGstreamer(camera_id, width, height, fps, flip_method), cv::CAP_GSTREAMER);

    if(!cap.isOpened()) {
        std::cerr << "Fail to open" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while(true) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        std::string file_name = std::to_string(ms);
        cap.read(frame);
        if(frame.empty()) continue;
        cv::imshow("Image", frame);
        int key = cv::waitKey(1);
        if(key == 27) break; // ESC
        else if(key == 115){ // s
            cv::imwrite(img_save_dir+file_name+".png", frame);
            std::cout << "Save " << img_save_dir+file_name+".png" << std::endl;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}