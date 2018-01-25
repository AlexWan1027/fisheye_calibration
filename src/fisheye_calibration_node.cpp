#include "fisheye_calibration.h"
#include "config.h"

bool select_img_flag;
    
void MouseEvent(int event, int x, int y, int flags, void* data)
{
    if(event == CV_EVENT_LBUTTONDBLCLK)
        select_img_flag = true;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fisheye_calibration_node");
    ros::NodeHandle nh;
    cv::VideoCapture cap;
    cv::Mat img;
    cap.open(0);
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1000);
    
    if(!cap.isOpened())
    {
        ROS_INFO("cannot open video");
	return -1;
    }
    
    select_img_flag = false;
    
    fisheye::Config myconfig;
    fisheye::FisheyeCalibration myCalibra(myconfig.board_size, myconfig.square_size, myconfig.success_thres);
    
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        if(!cap.read(img)) 
	{
	    ROS_INFO("cannot open video");
	}
// 	std::cout << "img" << img.rows << std::endl;
	cv::namedWindow("orignal img");
	cv::setMouseCallback("orignal img", MouseEvent, 0);
	cv::imshow("orignal img", img);
	cv::waitKey(1);
	
	if(select_img_flag == true)
	{
	    if(myCalibra.success_image < myconfig.success_thres)
	        myCalibra.processFrame(img);
	    else
	      ROS_INFO("all calibration steps have been finished");
	    select_img_flag = false;
	}
	
 	//ROS_ERROR("%d", select_img_flag);
	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}