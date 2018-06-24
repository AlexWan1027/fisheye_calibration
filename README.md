# The calibration tools for fisheye camera
===
This is the calibration project for fisheye camera which based on OpenCV finished by Alex Wan. If you want to know more information about principle, please visit to the following website:
[csdn blog](https://blog.csdn.net/bigdog_1027/article/details/78904526)

# 1. Build the project
```
    cd ~/catkin_ws/src
    git clone https://github.com/AlexWan1027/fisheye_calibration.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
# 2. Run the project
```
    roscore & 
    rosrun fisheye_calibration fisheye_calibration_node 
```

# 3. Perform the calibration task
The interface will appear a window showing the camera video, and double click the left mouse button in the window. Then the terminal will tell whether extract corners in the frame successfully. If successful, continue to click the mouse while changing the angle of the calibration board. After successfully extracting a number of images, it will automatically calculate the calibration result then saved in the file of YML.
