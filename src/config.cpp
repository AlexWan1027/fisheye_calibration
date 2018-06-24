#include "config.h"

namespace fisheye {

Config::Config()
{
    config_file = "/home/alex/myfile/fisheye/calibration/config.yml";
    cv::FileStorage fs0(config_file, cv::FileStorage::READ);
    board_size.width = fs0["board_width"];
    board_size.height = fs0["board_height"];
    square_size.height = fs0["square_size"];
    square_size.width = fs0["square_size"];
    success_thres = fs0["success_thres"];
    initial_guess_fxfy = fs0["initial_guess_fxfy"];
    initial_guess_u = fs0["initial_guess_u"];
    initial_guess_v = fs0["initial_guess_v"];
    std::cout << "I am in Config()" << success_thres << std::endl;
}

Config::~Config()
{
    ROS_INFO("Destroy the config");
}

} // namespace svo

