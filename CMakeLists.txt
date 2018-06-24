cmake_minimum_required(VERSION 2.8.3)
project(fisheye_calibration)

add_compile_options(-std=c++11)

#set(OpenCV_DIR /usr/local/share/OpenCV)
set(OpenCV_DIR /home/alex/Thirdparty/opencv-2.4.13/release)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
#   cv_bridge
  roscpp
  std_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${fisheye_calibration_SOURCE_DIR}/include/fisheye_calibration
)


add_executable(${PROJECT_NAME}_node src/fisheye_calibration.cpp src/config.cpp src/fisheye_calibration_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)
