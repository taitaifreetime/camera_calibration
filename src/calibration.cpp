#include "camera_calibration/calibration.hpp"

CameraCalibration::CameraCalibration(
    const int &checker_width, 
    const int &checker_height, 
    const double &square_size
):
board_size_(cv::Size(checker_width, checker_height)), 
square_size_(square_size)
{}

CameraCalibration::~CameraCalibration(){}


/**
 * @brief load images from dir
 * 
 * @param dir 
 * @param cv_imgs 
 * @return true 
 * @return false if dir not found
 */
bool CameraCalibration::loadImgs(
    const std::string &dir, 
    std::vector<cv::Mat> &cv_imgs
) const {
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

bool CameraCalibration::loadParameter(
    const std::string &file_name, 
    int &width, int &height, 
    std::string &camera_name, 
    cv::Mat &camera_matrix, 
    std::string &distortion_model, 
    cv::Mat &dist_coeffs, 
    cv::Mat &rectification_matrix, 
    cv::Mat &projection_matrix
) {
    auto loadMatrix = [](const YAML::Node &config, const std::string &matrix_name, cv::Mat &matrix) -> bool
    {
        try 
        {
            int rows = config[matrix_name]["rows"].as<int>();
            int cols = config[matrix_name]["cols"].as<int>();
            std::vector<double> data = config[matrix_name]["data"].as<std::vector<double>>();
            matrix = cv::Mat(rows, cols, CV_64F, data.data()).clone();
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
    };

    try
    {
        YAML::Node config = YAML::LoadFile(file_name);
        width = config["image_width"].as<int>();
        height = config["image_height"].as<int>();
        camera_name = config["camera_name"].as<std::string>();
        distortion_model = config["distortion_model"].as<std::string>();
        loadMatrix(config, "camera_matrix", camera_matrix);
        loadMatrix(config, "distortion_coefficients", dist_coeffs);
        loadMatrix(config, "rectification_matrix", rectification_matrix);
        loadMatrix(config, "projection_matrix", projection_matrix);

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 
 * 
 * @param imgs 
 */
void CameraCalibration::calibrateMonocularCamera(
    const std::vector<cv::Mat> &imgs
) const {
    if (!checkRequirements(imgs)) return;

    std::vector<std::vector<cv::Point3f>> object_points; // 3D points in real-world space
    std::vector<std::vector<cv::Point2f>> image_points;  // 2D points in image plane

    // Detect corners in all images
    std::cout << std::endl << "Corner Detection" << std::endl;
    int cnt = 0;
    for (const auto &img : imgs) 
    {
        if (img.empty()) continue;

        cv::Mat gray;
        if (img.channels() == 3) cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else                     gray = img.clone();

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray, board_size_, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) 
        {
            cnt++;
            cv::cornerSubPix(
                gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            image_points.push_back(corners);
            std::vector<cv::Point3f> obj;
            for (int i = 0; i < board_size_.height; i++) {
                for (int j = 0; j < board_size_.width; j++) {
                    obj.emplace_back(j * square_size_, i * square_size_, 0);
                }
            }
            object_points.push_back(obj);
            
#ifdef DEBUG
            cv::Mat drawn_img = img.clone();
            cv::drawChessboardCorners(drawn_img, board_size_, corners, found);
            cv::imshow("Detected Corners", drawn_img);
            cv::waitKey(0); 
#endif

            std::cout << "Checkerboard found: " << cnt << std::endl;
        } 
        else std::cerr << "[Error] Checkerboard not found in an image" << std::endl;
    }

    if (image_points.empty()) 
    {
        std::cerr << "[Error] No valid checkerboard detected in any image!" << std::endl;
        return;
    }

    std::cout << std::endl << "Calibrating ..." << std::endl;
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat rectification_matrix = cv::Mat::eye(3, 3, CV_64F); 
    cv::Mat projection_matrix = cv::Mat::zeros(3, 4, CV_64F); 

    double rms = cv::calibrateCamera(
        object_points, image_points, imgs[0].size(),
        camera_matrix, dist_coeffs, rvecs, tvecs);
    camera_matrix.copyTo(projection_matrix(cv::Rect(0, 0, 3, 3)));

    std::cout << std::endl;
    std::cout << "Calibration RMS error: " << rms << std::endl;
    std::cout << "Camera matrix:\n" << camera_matrix << std::endl;
    std::cout << "Distortion coefficients:\n" << dist_coeffs << std::endl;

    // save to file
    std::ofstream ofs("../monocular_camera_parameters.yaml");
    if (!ofs.is_open()) {
        std::cerr << "[Error] Cannot open file for writing." << std::endl;
        return;
    }
    ofs << formatParameter(
        imgs[0].cols, imgs[0].rows,
        camera_matrix, dist_coeffs, 
        rectification_matrix, projection_matrix
    );
    ofs.close();
}


/**
 * @brief 
 * 
 * @param left_imgs 
 * @param right_imgs 
 */
void CameraCalibration::calibrateStereoCamera(
    const std::vector<cv::Mat> &left_imgs, 
    const std::vector<cv::Mat> &right_imgs
) const {
    if (!checkRequirements(left_imgs, right_imgs)) return;

    std::vector<std::vector<cv::Point3f>> object_points;  // 3D points
    std::vector<std::vector<cv::Point2f>> left_image_points, right_image_points;

    std::cout << std::endl << "Corner Detection" << std::endl;
    int cnt = 0;
    for (size_t i = 0; i < left_imgs.size(); ++i) 
    {
        const auto &left_img = left_imgs[i];
        const auto &right_img = right_imgs[i];

        if (left_img.empty() || right_img.empty()) continue;

        cv::Mat gray_left, gray_right;
        if (left_img.channels() == 3) cv::cvtColor(left_img, gray_left, cv::COLOR_BGR2GRAY);
        else gray_left = left_img.clone();

        if (right_img.channels() == 3) cv::cvtColor(right_img, gray_right, cv::COLOR_BGR2GRAY);
        else gray_right = right_img.clone();

        std::vector<cv::Point2f> corners_left, corners_right;

        bool found_left = cv::findChessboardCorners(
            gray_left, board_size_, corners_left,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        bool found_right = cv::findChessboardCorners(
            gray_right, board_size_, corners_right,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found_left && found_right) 
        {
            cnt++;
            cv::cornerSubPix(
                gray_left, corners_left, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

            cv::cornerSubPix(
                gray_right, corners_right, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

            left_image_points.push_back(corners_left);
            right_image_points.push_back(corners_right);
            std::vector<cv::Point3f> obj;
            for (int i = 0; i < board_size_.height; i++) {
                for (int j = 0; j < board_size_.width; j++) {
                    obj.emplace_back(j * square_size_, i * square_size_, 0);
                }
            }
            object_points.push_back(obj);

#ifdef DEBUG
            cv::Mat drawn_left_img = left_img.clone();
            cv::drawChessboardCorners(drawn_left_img, board_size_, corners_left, found_left);
            cv::Mat drawn_right_img = right_img.clone();
            cv::drawChessboardCorners(drawn_right_img, board_size_, corners_right, found_right);
            cv::Mat corner_combined;
            cv::hconcat(drawn_left_img, drawn_right_img, corner_combined);
            cv::resize(corner_combined, corner_combined, cv::Size(), 0.7, 0.7);
            cv::imshow("left to right corners", corner_combined);
            cv::waitKey(0); 
#endif

            std::cout << "Checkerboard found:" << cnt << std::endl;
        } 
        else std::cerr << "[Warning] Checkerboard not found in image pair index " << i << std::endl;
    }

    if (left_image_points.empty() || right_image_points.empty()) {
        std::cerr << "[Error] No valid stereo checkerboard pairs detected!" << std::endl;
        return;
    }

    // Intrinsic matrices for left and right cameras (start as identity)
    cv::Mat left_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat right_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat left_dist_coeff = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat right_dist_coeff = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat R, T, E, F;
    
    std::cout << "Calibrating left camera ..." << std::endl;
    double left_rms = cv::calibrateCamera(
        object_points, left_image_points, left_imgs[0].size(),
        left_camera_matrix, left_dist_coeff, rvecs, tvecs);
    std::cout << "Calibration RMS error: " << left_rms << std::endl;
    std::cout << "Camera matrix:\n" << left_camera_matrix << std::endl;
    std::cout << "Distortion coefficients:\n" << left_dist_coeff << std::endl;
    std::cout << std::endl;

    std::cout << "Calibrating right camera ..." << std::endl;
    double right_rms = cv::calibrateCamera(
        object_points, right_image_points, right_imgs[0].size(),
        right_camera_matrix, right_dist_coeff, rvecs, tvecs);
    std::cout << "Calibration RMS error: " << right_rms << std::endl;
    std::cout << "Camera matrix:\n" << right_camera_matrix << std::endl;
    std::cout << "Distortion coefficients:\n" << right_dist_coeff << std::endl;
    std::cout << std::endl;

    std::cout << "Calibrating stereo camera system ..." << std::endl;
    double rms = cv::stereoCalibrate(
        object_points,
        left_image_points, right_image_points,
        left_camera_matrix, left_dist_coeff,
        right_camera_matrix, right_dist_coeff,
        left_imgs[0].size(),
        R, T, E, F,
        cv::CALIB_USE_INTRINSIC_GUESS,  
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-5)
    );
    cv::Mat R_left, R_right, P_left, P_right, Q; 
    cv::stereoRectify(
        left_camera_matrix, left_dist_coeff, 
        right_camera_matrix, right_dist_coeff, 
        left_imgs[0].size(),
        R, T, R_left, R_right, P_left, P_right, Q);

    std::cout << std::endl;
    std::cout << "Stereo Calibration RMS error: " << rms << std::endl;
    std::cout << "Left Camera Matrix:\n" << left_camera_matrix << std::endl;
    std::cout << "Right Camera Matrix:\n" << right_camera_matrix << std::endl;
    std::cout << "Rotation:\n" << R << std::endl;
    std::cout << "Translation:\n" << T << std::endl;

    // save to file
    std::ofstream ofs_left("../left_camera_parameters.yaml");
    ofs_left << formatParameter(
        left_imgs[0].cols, left_imgs[0].rows,
        left_camera_matrix, left_dist_coeff,
        R_left, P_left);
    ofs_left.close();

    std::ofstream ofs_right("../right_camera_parameters.yaml");
    ofs_right << formatParameter(
        right_imgs[0].cols, right_imgs[0].rows,
        right_camera_matrix, right_dist_coeff,
        R_right, P_right);
    ofs_right.close();
}

/**
 * @brief check prerequirements for monocular camera calibration
 * 
 * @param imgs calibration images
 * @return true if all requirements are true
 * @return false 
 */
bool CameraCalibration::checkRequirements(
    const std::vector<cv::Mat> &imgs
) const {
    if (imgs.size() < num_imgs_)
    {
        std::cerr << "[Error] The number of the images is not enough. " << num_imgs_ << " images required." << std::endl;
        return false;
    }
    // if ()
    return true;
}


/**
 * @brief check prerequirements for stereo camera calibration
 * 
 * @param left_imgs calibration images
 * @param right_imgs same
 * @return true if all requirements are true
 * @return false 
 */
bool CameraCalibration::checkRequirements(
    const std::vector<cv::Mat> &left_imgs, 
    const std::vector<cv::Mat> &right_imgs
) const {
    if (left_imgs.size() < num_imgs_ && right_imgs.size() < num_imgs_)
    {
        std::cerr << "[Error] The number of the images is not enough. " << num_imgs_ << " images required." << std::endl;
        return false;
    }
    if (left_imgs.size() != right_imgs.size())
    {
        std::cerr << "[Error] The numbers of the images do not match." << std::endl;
        return false;
    }
    return true;
}


/**
 * @brief format calibration parameters
 * 
 * @param img_width 
 * @param img_height 
 * @param K camera matrix
 * @param D distortion coefficients
 * @param R rectification matrix
 * @param P projection matrix
 * @return std::string string in yaml
 */
std::string CameraCalibration::formatParameter(
    const int &img_width, const int &img_height, 
    const cv::Mat &K, const cv::Mat &D, 
    const cv::Mat &R, const cv::Mat &P
) const {
    std::string given_parameters = 
        "image_width: "+std::to_string(img_width)+
        "\nimage_height: "+std::to_string(img_height)+
        "\ncamera_name: !!edit\n";

    std::string distortion_model = "distortion_model: plumb_bob\n";

    return given_parameters+
        formatMatrix("camera_matrix", K)+
        distortion_model+
        formatMatrix("distortion_coefficients", D.t())+
        formatMatrix("rectification_matrix", R)+
        formatMatrix("projection_matrix", P);
}


/**
 * @brief format matrix parameter
 * 
 * @param param_type parameter name
 * @param matrix matrix parameter
 * @return std::string 
 */
std::string CameraCalibration::formatMatrix(
    const std::string &param_type, 
    const cv::Mat &matrix
) const {
    std::string str = 
        param_type+
        ":\n  rows: " + std::to_string(matrix.rows) + 
        "\n  cols: " + std::to_string(matrix.cols) + 
        "\n  data: [";
    for (int i = 0; i < matrix.rows; i++) 
    {
        for (int j = 0; j < matrix.cols; j++) 
        {
            str += std::to_string(matrix.at<double>(i, j));
            if (!(i == matrix.rows - 1 && j == matrix.cols - 1)) str += ", ";
        }
    }
    str += "]\n";

    return str;
}