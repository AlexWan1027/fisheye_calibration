#include "fisheye_calibration.h"

namespace fisheye{

FisheyeCalibration::FisheyeCalibration(cv::Size board_size_, cv::Size square_size_, int success_thres_):
  board_size(board_size_),
  square_size(square_size_),
  success_thres(success_thres_),
  success_image(0)
{
    object_Points.clear();
    corners.clear();
    corners_Seq.clear();
    point_counts.clear();
}

FisheyeCalibration::~FisheyeCalibration()
{
    ROS_INFO("Destroying irobotDetect......");
}

void FisheyeCalibration::processFrame(const cv::Mat &img)
{
    frame = img.clone();

    if(frame.channels()==3)
    {
        cv::cvtColor(frame, frame, CV_BGR2GRAY);
    }
    
    image_size = frame.size();
    
    bool patternfound = cv::findChessboardCorners(frame, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
            cv::CALIB_CB_FAST_CHECK);
    if (!patternfound)
    {
	ROS_INFO("cannot find corners");
	return;
    }
    else
    {
        cv::TermCriteria Term(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);
        cv::cornerSubPix(frame, corners, cv::Size(11, 11), cv::Size(-1, -1), Term);
	
	imageTemp = frame.clone();
	for (int j = 0; j < corners.size(); j++)
	{
	    cv::circle(imageTemp, corners[j], 10, cv::Scalar(0, 0, 255), 2, 8, 0);
	}
	
	success_image = success_image + 1;
	
	corners_Seq.push_back(corners);
    }
    if(success_image == success_thres)
        getBoardPoints();
    
    showImg(img, frame);
    return;
}

void FisheyeCalibration::getBoardPoints()
{
    for (int t = 0; t < success_image; t++)
    {
        std::vector<cv::Point3f> tempPointSet;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                /* 假设定标板放在世界坐标系中z=0的平面上 */
                cv::Point3f tempPoint;
                tempPoint.x = i*square_size.width;
                tempPoint.y = j*square_size.height;
                tempPoint.z = 0;
                tempPointSet.push_back(tempPoint);
            }
        }
        object_Points.push_back(tempPointSet);
    }
    
    for (int i = 0; i < success_image; i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    doCalibration();
    
    return;
}

void FisheyeCalibration::doCalibration()
{
    std::ofstream fout("caliberation_result.txt");
    flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flags |= cv::fisheye::CALIB_CHECK_COND;
    flags |= cv::fisheye::CALIB_FIX_SKEW;
    cv::fisheye::calibrate(object_Points, corners_Seq, image_size, 
			   intrinsic_matrix, distortion_coeffs, rotation_vectors, 
			   translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
    ROS_INFO("calibration is ok!!!");
  
    cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    fout << "相机内参数矩阵：" << std::endl;
    fout << intrinsic_matrix << std::endl;
    fout << "畸变系数：\n";
    fout << distortion_coeffs << std::endl;
    fout << std::endl;
    ROS_INFO("calibration's result is saved!!!");
    
    return;
}

void FisheyeCalibration::showImg(const cv::Mat &img, const cv::Mat &processimg)
{
    cv::namedWindow("orginal img", CV_WINDOW_AUTOSIZE);
    cv::imshow("orginal img", img);
    cv::waitKey(1);
    
    return;
  
}

}

