cmake_minimum_required(VERSION 3.10)
# Project name
project(camera_imu_lidar_calibration)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "RELEASE")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native ")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find catkin (the ROS build system)
find_package(catkin QUIET COMPONENTS
        roscpp
        rosbag
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        ndt_omp
        tf
        imuPacket
        pcl_ros
        pcl_conversions
        cv_bridge
)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(OpenCV REQUIRED)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs imuPacket
            INCLUDE_DIRS src
            LIBRARIES camera_imu_lidar_calibration_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        #        gtsam
)

##################################################
# Make the core library
##################################################
add_library(camera_imu_lidar_calibration_lib SHARED
            src/init/InertialInitializer.cpp
            src/state/State.cpp
            src/state/StateHelper.cpp
            src/state/Propagator.cpp
            src/core/calibManager.cpp
            src/update/UpdaterLidarOdometry.cpp
            src/update/UpdaterCameraTracking.cpp
            src/track/lidarOdometry.cpp
            src/track/cameraPoseTracking.cpp
            src/track/lidarPlaneDetector.cpp
            src/update/UpdaterCameraLidarConstraint.cpp
)
target_link_libraries(camera_imu_lidar_calibration_lib ${thirdparty_libraries})
target_include_directories(camera_imu_lidar_calibration_lib PUBLIC src)

##################################################
# Adding different executables
##################################################
add_executable(ros_test_camera_imu_lidar_calibration_node
               src/ros_camera_imu_lidar_calibration_test.cpp)
target_link_libraries(ros_test_camera_imu_lidar_calibration_node
                      camera_imu_lidar_calibration_lib ${catkin_LIBRARIES})

add_executable(ros_camera_imu_lidar_calibration_bag_generator_node src/ros_camera_imu_lidar_calibration_bag_generator.cpp)
target_link_libraries(ros_camera_imu_lidar_calibration_bag_generator_node ${catkin_LIBRARIES})





