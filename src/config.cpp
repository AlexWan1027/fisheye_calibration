#include "config.h"

namespace fisheye {

Config::Config():
  board_size(),
  square_size(),
  success_thres()
{}

Config::~Config()
{
    ROS_INFO("Destroy the config");
}

} // namespace svo

