#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver/CustomMsg.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include "gpsTool.hpp"
#include "common/math_utils.h"
#include "common/eigen_types.h"

using namespace std;

typedef pcl::PointXYZI PointType;
typedef std::numeric_limits< double > dbl;

enum class SensorType { VELODYNE, OUSTER, LIVOX, HESAI };

class ParamServer
{
public:
    ros::NodeHandle nh;

    std::string robot_id;

    // Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    // Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;



    // Save pcd
    bool savePCD;
    string savePCDDirectory;


    // Lidar Sensor Configuration
    int livoxCustomPoint;
    SensorType lidar_type;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imu_type;
    float imu_frequency;
    vector<double> imu_acc_noise_temp;
    vector<double> imu_gyro_noise_temp;
    vector<double> imu_ba_temp;
    vector<double> imu_bg_temp;
    vector<double> imu_gravity_temp;

    Eigen::Vector3d imu_acc_noise;
    Eigen::Vector3d imu_gyro_noise;
    Eigen::Vector3d imu_ba;
    Eigen::Vector3d imu_bg;
    Eigen::Vector3d imu_gravity;
    bool  use_imu_init;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    double step_max_;
    double step_min_;
    double zero_max_;
    double zero_min_;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float edge_leaf_size;
    float surf_leaf_size;


    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;





    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("pplio/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("pplio/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("pplio/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("pplio/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("pplio/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("pplio/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("pplio/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("pplio/mapFrame", mapFrame, "map");

        nh.param<bool>("pplio/savePCD", savePCD, false);
        nh.param<std::string>("pplio/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");


        std::string sensorStr;
        nh.param<int>("pplio/livoxCustomPoint", livoxCustomPoint, 0);
        nh.param<std::string>("pplio/lidar_type", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            lidar_type = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            lidar_type = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            lidar_type = SensorType::LIVOX;
        }
        else if (sensorStr == "hesai")
        {
            lidar_type = SensorType::HESAI;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid lidar_type type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            ros::shutdown();
        }
        nh.param<int>("pplio/N_SCAN", N_SCAN, 16);
        nh.param<int>("pplio/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("pplio/downsampleRate", downsampleRate, 1);
        nh.param<float>("pplio/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("pplio/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<int>("pplio/imu_type", imu_type, 0);
        nh.param<float>("pplio/imu_frequency", imu_frequency, 200);
        nh.param<vector<double>>("pplio/imu_acc_noise", imu_acc_noise_temp, vector<double>());
        nh.param<vector<double>>("pplio/imu_gyro_noise", imu_gyro_noise_temp, vector<double>());
        nh.param<vector<double>>("pplio/imu_ba", imu_ba_temp, vector<double>());
        nh.param<vector<double>>("pplio/imu_bg", imu_bg_temp, vector<double>());
        nh.param<vector<double>>("pplio/imu_gravity", imu_gravity_temp, vector<double>());
        nh.param<bool>("pplio/use_imu_init", use_imu_init, true);
        imu_acc_noise = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imu_acc_noise_temp.data(), 3, 1);
        imu_gyro_noise = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imu_gyro_noise_temp.data(), 3, 1);
        imu_ba = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imu_ba_temp.data(), 3, 1);
        imu_bg = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imu_bg_temp.data(), 3, 1);
        imu_gravity = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imu_gravity_temp.data(), 3, 1);

        nh.param<double>("pplio/step_max", step_max_, 0.15);
        nh.param<double>("pplio/step_min", step_min_, 0.15);
        nh.param<double>("pplio/zero_max", zero_max_, 0.15);
        nh.param<double>("pplio/zero_min", zero_min_, 0.15);

        
        nh.param<vector<double>>("pplio/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("pplio/extrinsicRPY", extRPYV, vector<double>());

        nh.param<vector<double>>("pplio/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();
        nh.param<float>("pplio/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("pplio/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("pplio/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("pplio/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("pplio/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("pplio/edge_leaf_size", edge_leaf_size, 0.2);
        nh.param<float>("pplio/surf_leaf_size", surf_leaf_size, 0.2);



        nh.param<int>("pplio/numberOfCores", numberOfCores, 2);
        nh.param<double>("pplio/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("pplio/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("pplio/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("pplio/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);


        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        imu_out.header.frame_id = baselinkFrame;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                  imu_in.orientation.z);
        Eigen::Quaterniond q_final;
        if (imu_type == 0)
        {
            q_final = extQRPY;
        }
        else if (imu_type == 1)
            q_final = q_from * extQRPY;
        else
            std::cout << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;

        q_final.normalize();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(
                q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() +
                q_final.w() * q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

template <typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub, const T &thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template <typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template <typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    // 将PoseStamped msg 转换到 Stamped
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");

    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

std::string padZeros(int val, int num_digits = 6)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

#endif
