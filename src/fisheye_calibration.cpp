#include "fisheye_calibration.h"

namespace fisheye{

FisheyeCalibration::FisheyeCalibration():
  success_image(0),
  nameWindowName("calibration image"),
  showImgSize(300, 300),
  nShowImageSize(500),
  nSplitLineSize(15),
  nAroundLineSize(50),
  flags(0)
{
    object_Points.clear();
    corners.clear();
    corners_Seq.clear();
    point_counts.clear();
    imageSet.clear();
    intrinsic_K = (cv::Mat_<double>(3, 3) << myconfig.initial_guess_fxfy, 0, myconfig.initial_guess_u, 0, myconfig.initial_guess_fxfy, myconfig.initial_guess_v, 0, 0, 1);
    distortion_coeffs = cv::Mat_<double>(4, 1);
    createShowImgWindows(nameWindowName);
    readImgRead = 0;
    square_size = myconfig.square_size;
    board_size = myconfig.board_size;
    success_thres = myconfig.success_thres;
}

FisheyeCalibration::~FisheyeCalibration()
{
    ROS_INFO("Destroying irobotDetect......");
}

void FisheyeCalibration::processFrame(const cv::Mat &img)
{
     frame = img.clone();
//     sprintf(img_file, "/home/alex/myfile/data/fisheye_calibration/%d.jpg", readImgRead);
//     std::cout << img_file << std::endl;
//     frame = cv::imread(img_file);
//     readImgRead++;
  
    imageSet.clear();

    if(frame.channels()==3)
    {
        cv::cvtColor(frame, frame, CV_BGR2GRAY);
    }
    
    image_size = frame.size();
    std::cout << image_size.height << std::endl;
    imageSet.push_back(frame);
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
	
	std::string imageFileName;
        std::stringstream StrStm;
        StrStm << success_image;
	StrStm >> imageFileName;
        imageFileName += "_d.jpg";
	
        cv::imwrite(imageFileName, img);
	
	ROS_INFO("success = %d", success_image);
	imageSet.push_back(imageTemp);
	corners_Seq.push_back(corners);
	mergImage(imageSet);
	showImg(mergeImg, nameWindowName);
	
    }
    if(success_image == success_thres)
        getBoardPoints();
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
                tempPoint.x = i * square_size.width;
                tempPoint.y = j * square_size.height;
                tempPoint.z = 0;
                tempPointSet.push_back(tempPoint);
            }
        }
        object_Points.push_back(tempPointSet);
    }

    doCalibration();
    
    return;
}

void FisheyeCalibration::doCalibration()
{
    cv::FileStorage fs("fisheye.yml", cv::FileStorage::WRITE);

//     std::ofstream fout("caliberation_result.txt");
    if(myconfig.initial_guess_fxfy >= 0)
    {
	K = intrinsic_K;
	flags |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
    }
    flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flags |= cv::fisheye::CALIB_CHECK_COND;
    flags |= cv::fisheye::CALIB_FIX_SKEW;
    cv::fisheye::calibrate(object_Points, corners_Seq, image_size, 
 			   K, distortion_coeffs, rotation_vectors, 
 			   translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
    ROS_INFO("calibration is ok!!!");
  
    fs << "cameraMatrix" << K << "distCoeffs" << distortion_coeffs;  
    fs.release();
    
    ROS_INFO("calibration's result is saved!!!");
    
    return;
}

inline void FisheyeCalibration::showImg(const cv::Mat &img, std::string windowName)
{
    cv::imshow(windowName, img);
    cv::waitKey(1);
    return;
}

inline void FisheyeCalibration::mergImage(const std::vector<cv::Mat> &imgs)
{
    int nNumImages = imgs.size();
    cv::Size nSizeWindows = cv::Size(2, 1);
    
    imageHeight = nShowImageSize * nSizeWindows.width + nAroundLineSize + (nSizeWindows.width - 1) * nSplitLineSize;
    imageWidth = nShowImageSize * nSizeWindows.height + nAroundLineSize + (nSizeWindows.height - 1) * nSplitLineSize;
    
    cv::Mat showWindowImages(imageWidth, imageHeight, CV_8UC1, cv::Scalar::all(0));
    
    posX = (showWindowImages.cols - (nShowImageSize * nSizeWindows.width + (nSizeWindows.width - 1) * nSplitLineSize)) / 2;
    posY = (showWindowImages.rows - (nShowImageSize * nSizeWindows.height + (nSizeWindows.height - 1) * nSplitLineSize)) / 2;
    
    int tempPosX = posX;
    int tempPosY = posY;
    
    for(int i = 0; i < nNumImages; i++)
    {
        if((i % nSizeWindows.width == 0) && (tempPosX != posX))
	{
	    tempPosX = posX;
	    tempPosY += (nSplitLineSize + nShowImageSize);
	}
	cv::Mat tempImage = showWindowImages(cv::Rect(tempPosX, tempPosY, nShowImageSize, nShowImageSize));
	
	cv::resize(imgs[i], tempImage, cv::Size(nShowImageSize, nShowImageSize));
	
	tempPosX += (nSplitLineSize + nShowImageSize);
     }
     mergeImg = showWindowImages.clone();
}

inline void FisheyeCalibration::createShowImgWindows(std::string windowName)
{
    cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
    return;

}

}

