namespace utils
{
    namespace capture
    {
        std::string getGstreamer(const int cam_id, const int width, const int height, const int freq, const int flip_method)
        {
            return "nvarguscamerasrc sensor-id=" + std::to_string(cam_id) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
                std::to_string(height) + ", framerate=(fraction)" + std::to_string(freq) +
                "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(width) + ", height=(int)" +
                std::to_string(height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
        }
    }
    
}

