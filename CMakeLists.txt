cmake_minimum_required(VERSION 2.8.3)
project(pplio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 17)
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake) 
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  livox_ros_driver
)
set(OpenCV_DIR /home/liao/Tools/opencv-3.4.14/build)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(GeographicLib REQUIRED)
find_package(FMT REQUIRED)
find_package(TBB REQUIRED)  # 添加这行来找到TBB库

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)
set(OpenCV_DIR "/home/liao/Tools/opencv-3.4.14/build")# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}

)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${GeographicLib_LIBRARY_DIRS}

)

###########
## Build ##
###########
# Gps odometry
add_executable(${PROJECT_NAME}_gpsOdometry src/gpsOdom.cpp)
target_link_libraries(${PROJECT_NAME}_gpsOdometry ${catkin_LIBRARIES})

# # PDR odometry
# add_executable(${PROJECT_NAME}_pdrOdometry src/pdrOdom.cpp)
# target_link_libraries(${PROJECT_NAME}_pdrOdometry ${catkin_LIBRARIES})

# Range Image Projection
# add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
# add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# # Feature Association
# add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
# add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_eskfLIO
  src/ESKF_LIO.cpp
  src/Scancontext.cpp  
  src/dbscan.cpp
  src/imuProcess.cpp
  src/featureExtraction.hpp
  src/imageProjection.hpp
)
add_dependencies(${PROJECT_NAME}_eskfLIO ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_eskfLIO PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_eskfLIO Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ${GeographicLib_LIBRARIES} fmt::fmt TBB::tbb )

