// config.h

#ifndef CONFIG_H_
#define CONFIG_H_

#include "ros/ros.h"
#include <opencv2/opencv.hpp>

namespace fisheye {
  
class Config
{
public:
    Config();
    ~Config();

public:
    cv::Size square_size;  
    cv::Size board_size;
    int success_thres;
};

} 

#endif
