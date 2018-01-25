// fisheye_calibration.h
#ifndef FISHEYE_CALIBRATION_H_
#define FISHEYE_CALIBRATION_H_

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>

namespace fisheye{
  
class FisheyeCalibration
{
public:
     FisheyeCalibration(cv::Size board_size_, cv::Size square_size_, int success_thres_);
    ~FisheyeCalibration();
    
    void processFrame(const cv::Mat &img);
    void doCalibration();
    void getBoardPoints();
    
    inline void showImg(const cv::Mat &img, std::string windowName);
    inline void createShowImgWindows(std::string windowName);
    inline void mergImage(const std::vector<cv::Mat> &imgs);
    
public:
    cv::Matx33d intrinsic_matrix;    /*****    摄像机内参数矩阵    ****/
    cv::Vec4d distortion_coeffs;     /* 摄像机的4个畸变系数：k1,k2,k3,k4*/
    std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
    std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
    int success_image;
    
private:
    cv::Mat frame, imageTemp, mergeImg;
    
    cv::Size image_size;
    cv::Size square_size;
    cv::Size board_size;
    cv::Size showImgSize;
    
    std::vector<std::vector<cv::Point3f> >  object_Points;  
    std::vector<cv::Point2f> corners;                  
    std::vector<std::vector<cv::Point2f> > corners_Seq;
    std::vector<int>  point_counts;
    std::vector<cv::Mat> imageSet;
    

    int flags;
    int success_thres;
    int imageHeight;
    int imageWidth;
    int nShowImageSize;
    int nSplitLineSize;
    int nAroundLineSize;
    int posX, posY;
    
    double image_ratio;
    
    std::string nameWindowName;
};

}


#endif
