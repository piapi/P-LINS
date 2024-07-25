#pragma once
#ifndef IMUPROCESS_H
#define IMUPROCESS_H
#include "ros/ros.h"
#include "common/common_lib.h"
#include "common/math_utils.h"
#include <deque>
struct IMU
{
    IMU() = default;
    IMU(double t, const Vec3d &gyro, const Vec3d &acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};
using IMUPtr = std::shared_ptr<IMU>;

class StaticIMUInit
{
public:
    struct Options
    {
        Options() {}
        double init_time_seconds_ = 10.0;            // 静止时间
        int init_imu_queue_max_size_ = 20000;        // 初始化IMU队列最大长度
        int static_odom_pulse_ = 5;                  // 静止时轮速计输出噪声
        double max_static_gyro_var = 0.5;            // 静态下陀螺测量方差
        double max_static_acce_var = 0.05;           // 静态下加计测量方差
        double gravity_norm_ = 9.81;                 // 重力大小
        bool use_speed_for_static_checking_ = false; // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
    };

    /// 构造函数
    StaticIMUInit(Options options = Options()) : options_(options) {}

    /// 添加IMU数据
    bool AddIMU(const IMU &imu);
    bool AddIMU(double t, const Vec3d &gyro, const Vec3d &acce);
    /// 添加轮速数据
    // bool AddOdom(const Odom& odom);

    /// 判定初始化是否成功
    bool InitSuccess() const
    {
        return init_success_;
    }

    /// 获取各Cov, bias, gravity
    Vec3d GetCovGyro() const
    {
        return cov_gyro_;
    }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

private:
    /// 尝试对系统初始化
    bool TryInit();

    Options options_;                // 选项信息
    bool init_success_ = false;      // 初始化是否成功
    Vec3d cov_gyro_ = Vec3d::Zero(); // 陀螺测量噪声协方差（初始化时评估）
    Vec3d cov_acce_ = Vec3d::Zero(); // 加计测量噪声协方差（初始化时评估）
    Vec3d init_bg_ = Vec3d::Zero();  // 陀螺初始零偏
    Vec3d init_ba_ = Vec3d::Zero();  // 加计初始零偏
    Vec3d gravity_ = Vec3d::Zero();  // 重力
    bool is_static_ = false;         // 标志车辆是否静止
    std::deque<IMU> init_imu_deque_; // 初始化用的数据
    double current_time_ = 0.0;      // 当前时间
    double init_start_time_ = 0.0;   // 静止的初始时间
};

#endif