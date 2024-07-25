#include "utility.h"
#include "pplio/cloud_info.h"
#include "pplio/save_map.h"
#include "gpsTool.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Dense>
#include <fstream> // 头文件
#include "dbscan.h"
#include <execution>
#include "eskf.hpp"
#include "imuProcess.h"
#include "featureExtraction.hpp"
#include "imageProjection.hpp"
using namespace Eigen;

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
ofstream pathfile; // 定义输出流对象
string filePath = "/home/liao/pplio/src/pplio/output/path.txt";
ofstream scorefile; // 定义输出流对象
string scorePath = "/home/liao/pplio/src/pplio/output/score.txt";
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class mapOptimization : public ParamServer
{

public:
    ros::Publisher pub_map;
    ros::Publisher pub_odomtry;
    ros::Publisher pub_path;
    ros::Publisher pub_key_frame;
    ros::Publisher pub_imu_path;

    ros::Publisher pub_slam_info;

    ros::Subscriber sub_cloud;
    ros::Subscriber sub_imu;

    pplio::cloud_info cloud_info;

    pcl::PointCloud<PointType>::Ptr cur_edge; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr cur_surf; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr cloud_origins;
    pcl::PointCloud<PointType>::Ptr coeff_sellect;
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D; // 用来选取局部地图的

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> edge_flag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> surf_flag;
    std::deque<sensor_msgs::Imu> imu_buffer;                // imu数据缓冲
    std::deque<sensor_msgs::PointCloud2::Ptr> lidar_buffer; //

    pcl::PointCloud<PointType>::Ptr local_map_edge;
    pcl::PointCloud<PointType>::Ptr local_map_surf;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_edge;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_key_points;

    pcl::PointCloud<PointType>::Ptr global_map; // 下采样之后的全局地图
    ros::Time lidar_info_stamp;
    double cur_lidar_time;
    double last_key_time;
    double last_timestamp_imu = -1.0;   // 最近imu时间
    double last_timestamp_step = -1.0;  // 上一次脚步探测的时间
    double last_timestamp_lidar = -1.0; // 上一次lidar的时间
    double last_timestamp_tf = -2.0;
    double lidar_end_time = 0.0;

    float transformTobeMapped[6]; // roll pitch yaw x y z

    std::mutex mtx;
    mutex mtx_buffer;

    bool isDegenerate = false;
    cv::Mat matP;

    int cur_edge_num = 0;
    int cur_surf_num = 0;

    nav_msgs::Path global_path;
    nav_msgs::Path imu_path;

    Eigen::Affine3f transPointAssociateToMap;
    double surf_score_T;
    double surf_score_R;
    deque<point> data_R;
    deque<point> data_t;
    bool correct_R = false;
    bool correct_t = false;
    double norm_max_R = 0.0;
    double norm_max_t = 0.0;
    double deMinR = 9999.0;
    double deMint = 9999.0;
    double de_mint = 0.0;

    // vector<double>  prev(2,0.0);
    std::string save_pcd_directory;
    std::string save_RTs_directory;
    std::string save_degenerateRTs_directory;
    std::string save_degeneratePCD_directory;
    std::deque<pcl::PointCloud<PointType>::Ptr> edges_key;
    std::deque<pcl::PointCloud<PointType>::Ptr> surfs_key;
    SE3 lidar_pose;
    SE3 prev_kf_pose;
    SE3 cur_kf_pose;
    std::vector<SE3> key_poses;
    tf::TransformListener tf_listener;
    tf::StampedTransform lidar2Baselink;
    bool imu_inited = false, is_first_worning = true, is_observe_lidar = false, lidar_pushed = false;
    MeasureGroup measures_group;
    // 一些类
    std::shared_ptr<ESKF<double>> eskf_tool; // ESKF工具
    std::shared_ptr<PDR> pdr_tool;           // PDR
    std::shared_ptr<StaticIMUInit> imu_init_tool;
    std::shared_ptr<ImageProjection> image_projection;
    std::shared_ptr<FeatureExtraction> feature_extract;
    NavStated last_imu_pose;

    double last_pdr_pos = 0; // pdr上一次在b系下的位置
    double vel_in_b = 0.6;
    double sum_R = 0.0, sum_T = 0.0; // 用于检测当前点是否是离群点
    double sumSq_R = 0.0, sumSq_T = 0.0;

    mapOptimization()
    {

        pub_map = nh.advertise<sensor_msgs::PointCloud2>("pplio/mapping/map_global", 1);
        pub_odomtry = nh.advertise<nav_msgs::Odometry>("pplio/mapping/odometry", 1);
        pub_path = nh.advertise<nav_msgs::Path>("pplio/mapping/path", 1);
        pub_imu_path = nh.advertise<nav_msgs::Path>("/imu_path", 100000);
        if (lidar_type == SensorType::LIVOX && livoxCustomPoint == 1)
        {
            sub_cloud = nh.subscribe<livox_ros_driver::CustomMsg>(
                pointCloudTopic, 100, &mapOptimization::LivoxPCLHandler, this, ros::TransportHints().tcpNoDelay());
        }
        else
        {
            sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &mapOptimization::StandardPCLHandler, this, ros::TransportHints().tcpNoDelay());
        }
        pub_key_frame = nh.advertise<sensor_msgs::PointCloud2>("pplio/mapping/cloud_registered", 1);
        sub_imu = nh.subscribe(imuTopic, 200000, &mapOptimization::IMUHandler, this, ros::TransportHints().tcpNoDelay());

        pub_slam_info = nh.advertise<pplio::cloud_info>("pplio/mapping/slam_info", 1);
        imu_init_tool = std::make_shared<StaticIMUInit>();
        pdr_tool = std::make_shared<PDR>(extRPY);
        eskf_tool = std::make_shared<ESKF<double>>();
        feature_extract = std::make_shared<FeatureExtraction>();
        image_projection = std::make_shared<ImageProjection>();
        AllocateMemory();
        TF();
        // create directory and remove old files;
        // savePCDDirectory = std::getenv("HOME") + savePCDDirectory; // rather use global path
        int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
        unused = system((std::string("mkdir ") + savePCDDirectory).c_str());

        // saveSCDDirectory = savePCDDirectory + "SCDs/"; // SCD: scan context descriptor
        // unused = system((std::string("exec rm -r ") + saveSCDDirectory).c_str());
        // unused = system((std::string("mkdir -p ") + saveSCDDirectory).c_str());

        save_pcd_directory = savePCDDirectory + "Scans/";
        unused = system((std::string("exec rm -r ") + save_pcd_directory).c_str());
        unused = system((std::string("mkdir -p ") + save_pcd_directory).c_str());

        save_RTs_directory = savePCDDirectory + "RTs/"; // 保存关键帧的点云和位姿
        unused = system((std::string("exec rm -r ") + save_RTs_directory).c_str());
        unused = system((std::string("mkdir -p ") + save_RTs_directory).c_str());

        save_degeneratePCD_directory = savePCDDirectory + "degenerateScans/";
        unused = system((std::string("exec rm -r ") + save_degeneratePCD_directory).c_str());
        unused = system((std::string("mkdir -p ") + save_degeneratePCD_directory).c_str());

        save_degenerateRTs_directory = savePCDDirectory + "degenerateRTs/";
        unused = system((std::string("exec rm -r ") + save_degenerateRTs_directory).c_str());
        unused = system((std::string("mkdir -p ") + save_degenerateRTs_directory).c_str());
    }
    void TF()
    {
        // 如果lidar系与baselink系不同（激光系和载体系），需要外部提供二者之间的变换关系
        if (lidarFrame != baselinkFrame)
        {
            try
            {
                // 等待3s
                tf_listener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                // lidar系到baselink系的变换
                tf_listener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }
    }
    void AllocateMemory()
    {

        global_map.reset(new pcl::PointCloud<PointType>());

        cur_edge.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        cur_surf.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloud_origins.reset(new pcl::PointCloud<PointType>());
        coeff_sellect.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        edge_flag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        surf_flag.resize(N_SCAN * Horizon_SCAN);

        std::fill(edge_flag.begin(), edge_flag.end(), false);
        std::fill(surf_flag.begin(), surf_flag.end(), false);

        local_map_edge.reset(new pcl::PointCloud<PointType>());
        local_map_surf.reset(new pcl::PointCloud<PointType>());
        kdtree_key_points.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_edge.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_surf.reset(new pcl::KdTreeFLANN<PointType>());
        Eigen::Quaterniond quat_temp(extRot);
        quat_temp.normalize();
        cur_kf_pose = SE3(quat_temp.toRotationMatrix(), extTrans);
        SE3d2Trans(cur_kf_pose, transformTobeMapped);

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }
    bool IMUInit(const double timestamp, const Eigen::Vector3d gyro, Eigen::Vector3d acce)
    {

        if (imu_inited)
            return true;
        if (use_imu_init)
        {
            if (is_first_worning)
            {
                is_first_worning = false;
                ROS_WARN("--------wait IMU init------");
            }
            if (!imu_init_tool->InitSuccess())
            {
                imu_init_tool->AddIMU(timestamp, gyro, acce);
                return false;
            }
            // 需要IMU初始化
            // 读取初始零偏，设置ESKF
            ESKF<double>::Options options;
            // 噪声由初始化器估计
            options.gyro_var_ << sqrt(imu_init_tool->GetCovGyro()[0]), sqrt(imu_init_tool->GetCovGyro()[1]), sqrt(imu_init_tool->GetCovGyro()[2]);
            options.acce_var_ << sqrt(imu_init_tool->GetCovAcce()[0]), sqrt(imu_init_tool->GetCovAcce()[1]), sqrt(imu_init_tool->GetCovAcce()[2]);
            eskf_tool->SetInitialConditions(options, imu_init_tool->GetInitBg(), imu_init_tool->GetInitBa(), imu_init_tool->GetGravity());
            return true;
        }
        else
        {
            // 读取初始零偏，设置ESKF
            ESKF<double>::Options options;
            // 噪声由初始化器估计
            options.gyro_var_ = imu_gyro_noise;
            options.acce_var_ = imu_acc_noise;
            eskf_tool->SetInitialConditions(options, imu_bg, imu_ba, imu_gravity);
            std::cout << "bg = " << imu_bg.transpose()
                      << "\n ba = " << imu_ba.transpose() << "\n gyro sq = " << imu_gyro_noise.transpose()
                      << "\n acce sq = " << imu_acc_noise.transpose() << "\n grav = " << imu_gravity.transpose()
                      << "\n norm: " << imu_gravity.norm() << std::endl;
            return true;
        }
        return false;
    }
    void IMUHandler(const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imu_msg);
        double timestamp = thisImu.header.stamp.toSec();
        if (timestamp < last_timestamp_imu)
        {
            ROS_WARN("imu loop back, clear buffer");
        }
        last_timestamp_imu = timestamp;
        Eigen::Vector3d gyro(
            thisImu.angular_velocity.x,
            thisImu.angular_velocity.y,
            thisImu.angular_velocity.z);
        Eigen::Vector3d acce(
            thisImu.linear_acceleration.x,
            thisImu.linear_acceleration.y,
            thisImu.linear_acceleration.z);
        if (!IMUInit(timestamp, gyro, acce))
            return;
        if (!imu_inited)
        {
            eskf_tool->SetIMU(thisImu);
            eskf_tool->SetPVQ(extRPY);
            last_timestamp_step = timestamp;
            imu_inited = true;
            last_imu_pose = NavStated(last_timestamp_step, cur_kf_pose);
            return;
        }
        mtx_buffer.lock(); // 加锁
        imu_buffer.push_back(thisImu);
        mtx_buffer.unlock();
    }
    void Livox2Standerd(const livox_ros_driver::CustomMsg::ConstPtr &livox_msg_in, sensor_msgs::PointCloud2::Ptr &pcl_standerd)
    {
        pcl::PointCloud<PointXYZIRT> pcl_in;
        for (unsigned int i = 0; i < livox_msg_in->point_num; ++i)
        {
            PointXYZIRT pt;
            pt.x = livox_msg_in->points[i].x;
            pt.y = livox_msg_in->points[i].y;
            pt.z = livox_msg_in->points[i].z;
            pt.intensity = livox_msg_in->points[i].reflectivity;
            pt.ring = livox_msg_in->points[i].line;
            pt.time = (livox_msg_in->points[i].offset_time) / 1e9;
            pcl_in.push_back(pt);
        }
        // sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl_standerd.reset(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(pcl_in, *pcl_standerd);
        pcl_standerd->header = livox_msg_in->header;
        pcl_standerd->header.frame_id = lidarFrame;
        // *pcl_standerd = pcl_ros_msg;
    }
        // 去畸变
    void UndistortPcl(const MeasureGroup &measures, pcl::PointCloud<PointXYZIRT>::Ptr &pcl_out)
    {
        /*** add the imu of the last frame-tail to the of current frame-head ***/
        auto IMUpose = measures.imu_state;
        IMUpose.push_front(last_imu_pose);

        /*** sort point clouds by offset time ***/
        /*** undistort each lidar point (backward propagation) ***/
        auto pcl_in = measures.lidar;
        sort(pcl_in->points.begin(), pcl_in->points.end(), time_list);
        SE3 T_end = IMUpose.back().GetSE3();
        /// 将所有点转到结束时刻状态上
        std::for_each(std::execution::par_unseq, pcl_in->points.begin(), pcl_in->points.end(), [&](auto &pt)
                      {
        SE3 Ti = T_end;
        NavStated match;

        // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
        math::PoseInterp<NavStated>(
            measures.lidar_beg_time + pt.time, IMUpose, [](const NavStated &s) { return s.timestamp_; },
            [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

        Vec3d pi(pt.x, pt.y, pt.z);
        Vec3d p_compensate =  T_end.inverse() * Ti * pi;

        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2); });
        pcl_out = pcl_in;
        last_imu_pose = IMUpose.back();
    }

    //Lidar和IMU同步
    bool SyncPackages(MeasureGroup &measures)
    {
        if (lidar_buffer.empty() || imu_buffer.empty())
        {
            return false;
        }
        /*** push lidar frame ***/
        if (!lidar_pushed)
        {
            measures.lidar.reset(new pcl::PointCloud<PointXYZIRT>());
            pcl::fromROSMsg(*(lidar_buffer.front()), *(measures.lidar));
            measures.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();
            lidar_end_time = measures.lidar_beg_time + measures.lidar->back().time;
            lidar_pushed = true;
        }
        if (last_timestamp_imu < lidar_end_time)
        {
            return false;
        }
        /*** push imu data, and pop from buffer ***/
        double imu_time = imu_buffer.front().header.stamp.toSec();
        // measures.imu.clear();
        // while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        // {
        //     imu_time = imu_buffer.front().header.stamp.toSec();
        //     measures.imu.push_back(imu_buffer.front());
        //     imu_buffer.pop_front();
        // }
        measures.imu_state.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_buffer.front().header.stamp.toSec();
            eskf_tool->Predict(imu_buffer.front());
            Eigen::Vector3d gyro(
                imu_buffer.front().angular_velocity.x,
                imu_buffer.front().angular_velocity.y,
                imu_buffer.front().angular_velocity.z);
            Eigen::Vector3d acce(
                imu_buffer.front().linear_acceleration.x,
                imu_buffer.front().linear_acceleration.y,
                imu_buffer.front().linear_acceleration.z);
            double cur_time = imu_buffer.front().header.stamp.toSec();
            imu_buffer.pop_front();
            // eskf_tool->ObserveNHC();
            if (pdr_tool->PDRCore(cur_time, gyro, acce))
            {
                double curPos = 0;
                pdr_tool->GetPosInB(curPos);
                nav_msgs::Odometry odom_temp;
                pdr_tool->GetOdometry(odom_temp);
                Eigen::Vector3d tt(odom_temp.pose.pose.position.x, odom_temp.pose.pose.position.y, odom_temp.pose.pose.position.z);
                Eigen::Quaterniond rr(odom_temp.pose.pose.orientation.w, odom_temp.pose.pose.orientation.x, odom_temp.pose.pose.orientation.y, odom_temp.pose.pose.orientation.z);
                SE3 pos(rr.toRotationMatrix(), tt);
                std::random_device rd;  // 随机数生成器设备
                std::mt19937 gen(rd()); // 标准的mersenne_twister_engine
                std::normal_distribution<double> dist(0, 0.1);
                Vec3d vel(0, 0, 0);
                if (curPos != last_pdr_pos)
                {
                    vel << vel_in_b, 0, dist(gen);
                    // eskf_tool->ObservePDRSpeed(vel);
                }
                else
                {
                    // eskf_tool->ObservePDRSpeed(vel);
                    vel_in_b = 0;
                    // std::cout << "zero v" << std::endl;
                }
                last_pdr_pos = curPos;
                // eskf_tool->ObserveSE3(pos);
                eskf_tool->ObservePDRSpeed(vel);
            }
            PubIMUPath(imu_time, eskf_tool->GetNominalSE3().unit_quaternion(), eskf_tool->GetNominalSE3().translation());
            measures.imu_state.push_back(eskf_tool->GetNominalState());
        }
        lidar_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }
    void run()
    {
        double t1 = omp_get_wtime();
        if (!ExtraFeature())
            return;

        // 更新当前匹配结果的初始位姿
        // 当前帧下采样
        DownsampleCurrentScan();
        // 对点云配准进行优化问题构建求解
        double t2 = omp_get_wtime();
        if (!Scan2MapOptimization())
            return;
        // std::cout << "[~~~~~~~~~~~~ Scan2MapOptimization ]: time: " << omp_get_wtime() - t2 << std::endl;
        lidar_pose = Trans2SE3d(transformTobeMapped);
        if (surf_score_T != surf_score_T){
            surf_score_T = 1000000;
        }
            
        if (surf_score_R != surf_score_R)
            surf_score_R = 1000000;
        if(!correct_t || !correct_R){
             
        }
        // eskf_tool->ObserveSE3(lidar_pose, 1 / 100000, 1 / 100000);
        eskf_tool->ObserveSE3(lidar_pose, surf_score_T / 1000, surf_score_R / 10000);
        cur_kf_pose =  eskf_tool->GetNominalSE3();
        SE3d2Trans(cur_kf_pose, transformTobeMapped);
        // 根据配准结果确定是否是关键帧
        SaveKeyFramesAndFactor();
        // 将lidar里程计信息发送出去
        PublishOdometry();
        // 发送可视化点云信息
        PublishFrames();
        // std::cout<<"[~~~~~~~~~~~~ ALL ]: time: "<< omp_get_wtime() - t1<<std::endl;
    }
    void StandardPCLHandler(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg)
    {
        if (!imu_inited)
            return;
        // extract time stamp
        // 提取当前时间戳
        lidar_info_stamp = pcl_msg->header.stamp;
        cur_lidar_time = pcl_msg->header.stamp.toSec();
        sensor_msgs::PointCloud2::Ptr pcl_standerd(new sensor_msgs::PointCloud2(*pcl_msg));
        if (cur_lidar_time < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        last_timestamp_lidar = cur_lidar_time;
    }
    bool ExtraFeature()
    {
        // double t1 = omp_get_wtime();
        if (!SyncPackages(measures_group))
            return false;
        // 做去畸变
        pcl::PointCloud<PointXYZIRT>::Ptr pcl_undistortd;
        pcl_undistortd.reset(new pcl::PointCloud<PointXYZIRT>);
        sensor_msgs::PointCloud2::Ptr pcl_ros_msg;
        pcl_ros_msg.reset(new sensor_msgs::PointCloud2());
        // eskf_tool->UndistortPcl(measures_group,pcl_undistortd);
        UndistortPcl(measures_group, pcl_undistortd);
        pcl::toROSMsg(*pcl_undistortd, *pcl_ros_msg);
        // LiDAR预处理
        image_projection->StandardCloudHandler(pcl_ros_msg, cloud_info);
        // 提取点云的面点和线点
        feature_extract->laserCloudInfoHandler(cloud_info, cur_surf, cur_edge);
        // std::cout<<"[~~~~~~~~~~~~ Feature Extract ]: time: "<< omp_get_wtime() - t1<<std::endl;
        return true;
    }
    void LivoxPCLHandler(const livox_ros_driver::CustomMsg::ConstPtr &pcl_msg)
    {
        if (!imu_inited)
            return;
        // extract time stamp
        // 提取当前时间戳
        lidar_info_stamp = pcl_msg->header.stamp;
        cur_lidar_time = pcl_msg->header.stamp.toSec();
        sensor_msgs::PointCloud2::Ptr pcl_standerd;
        Livox2Standerd(pcl_msg, pcl_standerd);
        if (cur_lidar_time < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        if (cur_lidar_time - last_timestamp_lidar < 0.15)
            return;
        last_timestamp_lidar = cur_lidar_time;
        // extract info and feature cloud
        
        //IMU、lidar同步
        lidar_buffer.push_back(pcl_standerd);
        run();
    }

    void PointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr TransformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, SE3 &transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3d transCur = Eigen::Affine3d::Identity();
        transCur.linear() = transformIn.rotationMatrix();
        transCur.translation() = transformIn.translation();
// 使用openmp进行并行加速
#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    Eigen::Affine3f PclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f Trans2Affine3f(float *transformIn)
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }
    SE3 Trans2SE3d(float *transformIn)
    {
        Eigen::Affine3f lidar_temp = pcl::getTransformation(transformIn[3], transformIn[4],
                                                            transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
        Eigen::Quaterniond qu(lidar_temp.rotation().cast<double>());
        qu.normalize();
        return SE3(qu.toRotationMatrix(), lidar_temp.translation().cast<double>());
    }
    void SE3d2Trans(SE3 pose, float *transformIn)
    {
        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(pose.unit_quaternion().x(),
                                     pose.unit_quaternion().y(),
                                     pose.unit_quaternion().z(),
                                     pose.unit_quaternion().w()))
            .getRPY(roll, pitch, yaw);
        transformIn[0] = roll;
        transformIn[1] = pitch;
        transformIn[2] = yaw;
        transformIn[3] = pose.translation().x();
        transformIn[4] = pose.translation().y();
        transformIn[5] = pose.translation().z();
    }
    pcl::PointCloud<PointType>::Ptr VoxelFilterCloud(pcl::PointCloud<PointType>::Ptr cloud, float voxel_size)
    {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel.setInputCloud(cloud);

        pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>());
        voxel.filter(*output);
        return output;
    }
    void DownsampleCurrentScan()
    {
        // Downsample cloud from current scan
        cur_edge = VoxelFilterCloud(cur_edge, edge_leaf_size);
        cur_edge_num = cur_edge->size();

        cur_surf == VoxelFilterCloud(cur_surf, surf_leaf_size);
        cur_surf_num = cur_surf->size();
    }

    void UpdatePointAssociateToMap()
    {
        // 将欧拉角转换为eigen的对象
        transPointAssociateToMap = Trans2Affine3f(transformTobeMapped);
    }

    void CornerOptimization()
    {
        UpdatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cur_edge_num; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = cur_edge->points[i];
            PointAssociateToMap(&pointOri, &pointSel);
            kdtree_edge->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0)
            {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++)
                {
                    cx += local_map_edge->points[pointSearchInd[j]].x;
                    cy += local_map_edge->points[pointSearchInd[j]].y;
                    cz += local_map_edge->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++)
                {
                    float ax = local_map_edge->points[pointSearchInd[j]].x - cx;
                    float ay = local_map_edge->points[pointSearchInd[j]].y - cy;
                    float az = local_map_edge->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1)
                    {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        edge_flag[i] = true;
                    }
                }
            }
        }
    }

    void SurfOptimization()
    {
        // 标志位清空
        std::fill(surf_flag.begin(), surf_flag.end(), false);
        UpdatePointAssociateToMap();
        int effect_nums = 0; // lzb 20230109
        MatrixXd H_observality(cur_surf_num, 6);
        Eigen::VectorXd meas_vec_(cur_surf_num);
#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cur_surf_num; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = cur_surf->points[i];
            PointAssociateToMap(&pointOri, &pointSel);
            kdtree_surf->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;
            // 平面方程Ax+By+Cz+1=0
            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();
            // 最大距离不能超过1m
            if (pointSearchSqDis[4] < 1.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = local_map_surf->points[pointSearchInd[j]].x;
                    matA0(j, 1) = local_map_surf->points[pointSearchInd[j]].y;
                    matA0(j, 2) = local_map_surf->points[pointSearchInd[j]].z;
                }
                // 求解Ax=b这个超定方程
                matX0 = matA0.colPivHouseholderQr().solve(matB0);
                // 求出来的x就是这个平面的法向量
                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                // 归一化，将法向量模唱统一为1
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    // 每个点带入平面方程，计算点到平面的距离，如果距离大于0.2m认为这个平面曲率偏大，就是无效的平面
                    if (fabs(pa * local_map_surf->points[pointSearchInd[j]].x +
                             pb * local_map_surf->points[pointSearchInd[j]].y +
                             pc * local_map_surf->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }
                // 如果通过了平面的校验
                if (planeValid)
                {

                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                    // 计算当前点到平面的距离
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.9)
                    {
                        Vector3d point_this(pointOri.x, pointOri.y, pointOri.z);
                        Vector3d norm_vec(pa, pb, pc);
                        Eigen::Matrix3d point_crossmat;
                        point_crossmat<<SKEW_SYM_MATRX(point_this);
                        Eigen::Vector3d A(point_crossmat * eskf_tool->GetNominalSE3().rotationMatrix().transpose() * norm_vec);
                        H_observality.block<1, 6>(effect_nums++, 0) << VEC_FROM_ARRAY(A), pa, pb, pc;
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        surf_flag[i] = true;
                    }
                }
            }
        }

        MatrixXd H_observality_(effect_nums, 6);
        H_observality_ = H_observality.block(0, 0, effect_nums, 6);
        // 计算点云观测可不可观
        MatrixXd P_H_observality(6, 6);
        P_H_observality = H_observality_.transpose() * H_observality_;
        // 提取位置精度因子
        MatrixXd T_observality(3, 3);
        T_observality = P_H_observality.block(3, 3, 3, 3);
        Eigen::JacobiSVD<Eigen::MatrixXd> T_svd(T_observality, Eigen::ComputeThinU | Eigen::ComputeThinV);
        MatrixXd R_observality(3, 3);
        R_observality = P_H_observality.block(0, 0, 3, 3);
        Eigen::JacobiSVD<Eigen::MatrixXd> R_svd(R_observality, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // 计算指标
        // 计算最大特征值/最小特征值
        // Eigen::Matrix3f diag = svd.singularValues().asDiagonal();
        // 平移判断
        Eigen::VectorXd record_T(3, 1);
        for (int i = 0; i < 3; ++i)
        {
            record_T(i) = T_svd.singularValues()(i, 0);
        }
        MatrixXd::Index maxRow_T, maxCol_T;
        MatrixXd::Index minRow_T, minCol_T;
        double min_T = record_T.minCoeff(&minRow_T, &minCol_T);
        double max_T = record_T.maxCoeff(&maxRow_T, &maxCol_T);
        surf_score_T = abs(max_T) / abs(min_T);
        // 旋转判断
        Eigen::VectorXd record_R(3, 1);
        for (int i = 0; i < 3; ++i)
        {
            record_R(i) = R_svd.singularValues()(i, 0);
        }
        MatrixXd::Index maxRow_R, maxCol_R;
        MatrixXd::Index minRow_R, minCol_R;
        double min_R = record_R.minCoeff(&minRow_R, &minCol_R);
        double max_R = record_R.maxCoeff(&maxRow_R, &maxCol_R);
        surf_score_R = abs(max_R) / abs(min_R);
        // ROS_INFO("surf_score_R:%.4f,surf_score_T:%.4f", surf_score_R, surf_score_T);
    }
    // 将角点约束和面点约束统一到一起
    void CombineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < cur_edge_num; ++i)
        {
            // 只有标志位是true的时候才是有效约束
            if (edge_flag[i] == true)
            {
                cloud_origins->push_back(laserCloudOriCornerVec[i]);
                coeff_sellect->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < cur_surf_num; ++i)
        {
            if (surf_flag[i] == true)
            {
                cloud_origins->push_back(laserCloudOriSurfVec[i]);
                coeff_sellect->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        // 标志位清空
        std::fill(edge_flag.begin(), edge_flag.end(), false);
        std::fill(surf_flag.begin(), surf_flag.end(), false);
    }
    bool LMOptimization(int iterCount)
    {
        // 原始的loam代码是将lidar坐标系转到相机坐标系，这里把loam中代码拷贝过来了，但是为坐标系的统一，就先转到相机坐标系优化，然后结果转回lidar系
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = cloud_origins->size();
        if (laserCloudSelNum < 50)
        {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++)
        {
            // lidar -> camera
            pointOri.x = cloud_origins->points[i].y;
            pointOri.y = cloud_origins->points[i].z;
            pointOri.z = cloud_origins->points[i].x;
            // lidar -> camera
            coeff.x = coeff_sellect->points[i].y;
            coeff.y = coeff_sellect->points[i].z;
            coeff.z = coeff_sellect->points[i].x;
            coeff.intensity = coeff_sellect->points[i].intensity;
            // in camera
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }
        // 构建JTJ以及——JTe矩阵
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        // 求解增量
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
        // 判断是否有退化
        if (iterCount == 0)
        {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
            // 对JTJ进行特征分解
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};

            // for (int i = 5; i >= 0; i--)
            // {
            //     // 特征值从小大道遍历，如果小于阈值就认为退化
            //     if (matE.at<float>(0, i) < eignThre[i])
            //     {
            //         for (int j = 0; j < 6; j++)
            //         {
            //             // 对相应的特征向量置0
            //             matV2.at<float>(i, j) = 0;
            //         }
            //         isDegenerate = true;
            //     }
            //     else
            //     {
            //         break;
            //     }
            // }
            if (correct_R)
            {
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        // 对相应的特征向量置0
                        matV2.at<float>(i, j) = 0;
                    }
                }
                isDegenerate = true;
            }
            if (correct_t)
            {
                for (int i = 3; i < 6; ++i)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        // 对相应的特征向量置0
                        matV2.at<float>(i, j) = 0;
                    }
                }
                isDegenerate = true;
            }
            matP = matV.inv() * matV2;
        }
        // 如果发生退化，静对增量进行修改，退化方向不更新
        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }
        // 增量更新
        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);
        // 计算更新的旋转和平移大小
        float deltaR = sqrt(
            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));

        // 足够小就认为优化问题收敛了
        if (deltaR < 0.05 && deltaT < 0.05)
        {
            return true; // converged
        }
        // 否则继续 更新
        return false; // keep optimizing
    }

    bool Scan2MapOptimization()
    {
        // 没有关键帧，直接返回
        if (key_poses.empty())
            return true;
        // 判断当前帧面点和角点数量是否足够
        SE3d2Trans(eskf_tool->GetNominalSE3(),transformTobeMapped);
        if (cur_edge_num > edgeFeatureMinValidNum || cur_surf_num > surfFeatureMinValidNum)
        {
            // 分别把角点和面点构建kdtree
            kdtree_edge->setInputCloud(local_map_edge);
            kdtree_surf->setInputCloud(local_map_surf);
            // 迭代求解
            for (int iterCount = 0; iterCount < 10; iterCount++)
            {
                cloud_origins->clear();
                coeff_sellect->clear();
                // CornerOptimization();
                SurfOptimization();
                 // if (iterCount == 0)
                // {
                //     correct_R = false;
                //     correct_t = false;
                //     if (IsOutlier(surf_score_R, data_R, sum_R, sumSq_R, 200) && surf_score_R > norm_max_R + 30)
                //     {
                //         correct_R=true;
                //         cout<<"R degeneracy:"<<surf_score_R<<endl;
                //     }
                //     else
                //     {
                //         AddData(surf_score_R, data_R, sum_R, sumSq_R, 600);
                //         norm_max_R=surf_score_R;
                //     }
                //     if (IsOutlier(surf_score_T, data_t, sum_T, sumSq_T, 200) && norm_max_t < surf_score_T - 8)
                //     {
                //         correct_t=true;
                //         cout << "T degeneracy:" << surf_score_T << endl;
                //     }
                //     else
                //     {
                //         AddData(surf_score_T, data_t, sum_T, sumSq_T, 600);
                //         norm_max_t = surf_score_T;
                //     }
                // }
                // if (iterCount == 0)
                // {
                //     double t1 = omp_get_wtime();
                //     correct_R = false;
                //     if (data_R.size() >= 400)
                //     {
                //         data_R.pop_front();
                //     }
                //     if (data_R.size() <= 50)
                //     {
                //         data_R.push_back(surf_score_R);
                //     }
                //     else
                //     {
                //         data_R.push_back(surf_score_R);
                //         DBSCAN(data_R, select_MinPts(data_R, 3), 3);
                //         int len = data_R.size();
                //         if (data_R[len - 1].pointType == 0 && surf_score_R > norm_max_R + 80 /*|| surf_score_R > deMinR*/)
                //         {
                //             deMinR = min(surf_score_R, deMinR);
                //             correct_R = true;
                //             cout << "R degeneracy:" << surf_score_R << endl;
                //         }
                //     }
                //     correct_t = false;
                //     if (data_t.size() >= 400)
                //     {
                //         data_t.pop_front();
                //     }
                //     if (data_t.size() <= 50)
                //     {
                //         data_t.push_back(surf_score_T);
                //     }
                //     else
                //     {
                //         data_t.push_back(surf_score_T);
                //         DBSCAN(data_t, select_MinPts(data_t, 3), 3);
                //         int len = data_t.size();
                //         if (data_t[len - 1].pointType == 0 && surf_score_T > norm_max_t + 5 /*|| surf_score_T > deMint*/)
                //         {
                //             deMint = min(surf_score_T, deMint);
                //             correct_t = true;
                //             cout << "T degeneracy:" << surf_score_T << endl;
                //         }
                //     }
                //     if (!correct_R)
                //     {
                //         norm_max_R = max(norm_max_R, surf_score_R);
                //     }
                //     if (!correct_t)
                //     {
                //         norm_max_t = max(norm_max_t, surf_score_T);
                //     }
                //     scorefile << setprecision(20) << cur_lidar_time << "\t" << surf_score_T << "\t" << correct_t << "\t" << surf_score_R << "\t" << correct_R << endl;

                    // std::cout << "[~~~~~~~~~~~~ DBSCAN]: time: " << omp_get_wtime() - t1 << std::endl;
                // }
                CombineOptimizationCoeffs();
                if (LMOptimization(iterCount) == true)
                    break;
            }

            return true;
        }else{
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", cur_edge_num, cur_surf_num);
        }

        return false;
    }
    void AddData(double value, std::deque<double> &window, double &sum, double &sumSq, int windowSize)
    {
        window.push_back(value);
        sum += value;
        sumSq += value * value;

        if (window.size() > windowSize)
        {
            double old = window.front();
            window.pop_front();
            sum -= old;
            sumSq -= old * old;
        }
    }
    bool IsOutlier(double newValue, std::deque<double> &window,double &sum,double &sumSq,int windowSize)
    {
        if (window.size() < windowSize)
            return false;

        double mean = sum / window.size();
        double variance = (sumSq / window.size()) - (mean * mean);
        double stddev = std::sqrt(variance);

        // 这里的2可以调整，以适应不同的离群点定义
        return std::abs(newValue - mean) > 5 * stddev;
    }
    bool IsKeyframe(const SE3 &current_pose)
    {
        if (key_poses.empty())
            return true;
        // 只要与上一帧相对运动超过一定距离或角度，就记关键帧
        SE3 delta = prev_kf_pose.inverse() * current_pose;
        return delta.translation().norm() > 1.0 ||
               delta.so3().log().norm() > 15 * math::kDEG2RAD;
    }
    double CalculateVel()
    {
        double cur_x = cur_kf_pose.translation().x();
        double cur_y = cur_kf_pose.translation().y();
        double prev_x = prev_kf_pose.translation().x();
        double prev_y = prev_kf_pose.translation().y();
        double deltaX = (cur_x - prev_x) * (cur_x - prev_x);
        double deltaY = (cur_y - prev_y) * (cur_y - prev_y);
        return sqrt(deltaX + deltaY) / (cur_lidar_time - last_key_time);
    }
    void BuildLocalMap()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        local_map_edge.reset(new pcl::PointCloud<PointType>());
        local_map_surf.reset(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;     // 保存kdtree提取出来的元素的索引
        std::vector<float> pointSearchSqDis; // 保存距离查询位置的距离的数组
        kdtree_key_points->setInputCloud(cloudKeyPoses3D);
        kdtree_key_points->radiusSearch(cloudKeyPoses3D->back(), (double)(surroundingKeyframeSearchRadius), pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
           int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }
        surroundingKeyPoses = VoxelFilterCloud(surroundingKeyPoses, surroundingKeyframeDensity);
        for (auto &pt : surroundingKeyPoses->points)
        {
            kdtree_key_points->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
            int thisKeyInd = pt.intensity;
            *local_map_edge += *edges_key[thisKeyInd];
            *local_map_surf += *surfs_key[thisKeyInd];
        }
        local_map_surf = VoxelFilterCloud(local_map_surf, surf_leaf_size);
        local_map_edge = VoxelFilterCloud(local_map_edge, edge_leaf_size);
    }
    void SaveKeyFramesAndFactor()
    {
        // 通过旋转平移的增量来判读是否是关键帧
        if (!IsKeyframe(cur_kf_pose))
            return;
        vel_in_b = CalculateVel();
        last_key_time = cur_lidar_time;
        prev_kf_pose = cur_kf_pose;
        key_poses.push_back(cur_kf_pose);
        // // save key poses
        PointType thisPose3D;
        thisPose3D.x = cur_kf_pose.translation().x();
        thisPose3D.y = cur_kf_pose.translation().y();
        thisPose3D.z = cur_kf_pose.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);
        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*cur_edge, *thisCornerKeyFrame);
        pcl::copyPointCloud(*cur_surf, *thisSurfKeyFrame);

        pcl::PointCloud<PointType>::Ptr cur_edge_temp = TransformPointCloud(thisCornerKeyFrame, cur_kf_pose);
        pcl::PointCloud<PointType>::Ptr cur_surf_temp = TransformPointCloud(thisSurfKeyFrame, cur_kf_pose);
        edges_key.emplace_back(cur_edge_temp);
        surfs_key.emplace_back(cur_surf_temp);
        // *global_map += *cur_edge_temp;
        // *global_map += *cur_surf_temp;
        double t1 = omp_get_wtime();
        BuildLocalMap();
        // std::cout<<"[~~~~~~~~~~~~ BuildLocalMap ]: time: "<< omp_get_wtime() - t1<<std::endl;


        UpdatePath(cur_kf_pose);
    }
    void PubIMUPath(const double time,const Eigen::Quaterniond quat, const Eigen::Vector3d trans)
    {
        imu_path.header.frame_id = "map";
        imu_path.header.stamp = ros::Time().fromSec(time);
        geometry_msgs::PoseStamped pose;
        pose.header = imu_path.header;
        pose.pose.position.x = trans.x();
        pose.pose.position.y = trans.y();
        pose.pose.position.z = trans.z();
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();
        imu_path.poses.push_back(pose);
        if (imu_path.poses.size())
        {
            pub_imu_path.publish(imu_path);
        }
    }
    void UpdatePath(const SE3 &pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(cur_lidar_time);
        pose_stamped.header.frame_id = mapFrame;
        pose_stamped.pose.position.x = pose_in.translation().x();
        pose_stamped.pose.position.y = pose_in.translation().y();
        pose_stamped.pose.position.z = pose_in.translation().z();
        pose_stamped.pose.orientation.x = pose_in.unit_quaternion().x();
        pose_stamped.pose.orientation.y = pose_in.unit_quaternion().y();
        pose_stamped.pose.orientation.z = pose_in.unit_quaternion().z();
        pose_stamped.pose.orientation.w = pose_in.unit_quaternion().w();
        global_path.poses.push_back(pose_stamped);
    }

    void PublishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = lidar_info_stamp;
        laserOdometryROS.header.frame_id = mapFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pathfile << setprecision(20) << cur_lidar_time << " " << transformTobeMapped[3] << " " << transformTobeMapped[4] << " " << transformTobeMapped[5] << " "
                 << laserOdometryROS.pose.pose.orientation.x << " " << laserOdometryROS.pose.pose.orientation.y << " " << laserOdometryROS.pose.pose.orientation.z << " " << laserOdometryROS.pose.pose.orientation.w << endl;
        pub_odomtry.publish(laserOdometryROS);

        // Publish TF
        // static tf::TransformBroadcaster br;
        // tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
        //                                               tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        // tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, lidar_info_stamp, mapFrame, "lidar_link");
        // br.sendTransform(trans_odom_to_lidar);
        
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometryROS.pose.pose, tCur);
        if (lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, lidar_info_stamp, mapFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);
    }

    void PublishFrames()
    {
        if (key_poses.empty())
            return;
        // publish registered key frame
        if (pub_key_frame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            *cloudOut += *TransformPointCloud(cur_edge, cur_kf_pose);
            *cloudOut += *TransformPointCloud(cur_surf, cur_kf_pose);
            publishCloud(pub_key_frame, cloudOut, lidar_info_stamp, mapFrame);
        }
        // publish gloabalmap
        // if (pub_map.getNumSubscribers() != 0)
        // {
        //     publishCloud(pub_map, global_map, lidar_info_stamp, mapFrame);
        // }
        // publish path
        if (pub_path.getNumSubscribers() != 0)
        {
            global_path.header.stamp = lidar_info_stamp;
            global_path.header.frame_id = mapFrame;
            pub_path.publish(global_path);
        }

        // publish SLAM infomation for 3rd-party usage
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pplio");
        pathfile.open(filePath); // 打开文件
    if (!pathfile)
    {
        cout << "打开文件失败" << endl;
        exit(1);
    }
        scorefile.open(scorePath); // 打开文件
    if (!scorefile)
    {
        cout << "打开文件失败" << endl;
        exit(1);
    }
    mapOptimization MO;
    ROS_INFO("\033[1;32m---->ESKF_LIO.\033[0m");
    ros::Rate rate(5000);
    // while (ros::ok)
    // {
    //     /* code */
    //     MO.run();
        
    //     rate.sleep();
    // }
    ros::spin();
    return 0;
}