#include "config.h"

namespace fisheye {

Config::Config():
  board_size(8,6),
  square_size(28, 28),
  success_thres(5)
{}

Config::~Config()
{
    ROS_INFO("Destroy the config");
}

} // namespace svo

