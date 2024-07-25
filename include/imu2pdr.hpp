#pragma once
#ifndef PDR_H
#define PDR_H
#include <iostream>
#include <math.h>
#include <vector>
#include <deque>
#include <iomanip>
#include <fstream>
#include <utility>
#include "Eigen/Core"
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#define D2R 0.017453292519943295
class PDR : public ParamServer
{
public:
    PDR(Eigen::Matrix3d rot=Eigen::Matrix3d::Identity(), Eigen::Vector3d pos = Eigen::Vector3d::Zero(), double g_ = 9.8)
    {
        step_max = step_max_;
        step_min = step_min_;
        zero_max = zero_max_;
        zero_min = zero_min_;
        cout << "step:" << step_min << "," << step_max << "; zero_max:" << zero_min << "," << zero_max << endl;
        Eigen::Quaterniond quat(rot);
        tf::Matrix3x3(tf::Quaternion(quat.x(),
                                     quat.y(),
                                     quat.z(),
                                     quat.w()))
            .getRPY(attitude[0], attitude[1], attitude[2]);
        position=pos;
        pdr_inited = 0;
        g = g_;
        savefile.open("/home/liao/pplio/src/pplio/output/pdr_result.txt");
        if (!savefile.is_open())
        {
            printf("save imu file failed");
        }
    }
    // 脚步推算,并不一定是一个科学的做法
    double GetStep()
    {
        return step.second;
    }
    double StepStride(double step_max, double step_min)
    {
        double len = 0.425 * pow(abs(step_max - step_min), 0.25);
        return len;
    }
    // 计算roll和pitch
    void CalRollPitch(Eigen::Vector3d &euler, Eigen::Vector3d acc)
    {
        euler[1] = atan2(acc[1], acc[2]);
        euler[0] = -atan2(acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2]));
    }
    // 航向角积分量
    void CalYaw(Eigen::Vector3d gry, Eigen::Vector3d acc, Eigen::Vector3d &euler, double dt)
    {
        CalRollPitch(euler, acc);
        double cosr = cos(euler[0]);
        double cosp = cos(euler[1]);
        double sinr = sin(euler[0]);
        double sinp = sin(euler[1]);
        double yaw_diff = gry[1] * sinr / cosp + gry[2] * cosr / cosp;
        euler[2] += yaw_diff * dt;

        CentralYaw(euler[2]);
    }
    // 保证角度在-pi到pi之间
    void CentralYaw(double &yaw)
    {
        while (yaw > M_PI || yaw <= -M_PI)
        {
            if (yaw > M_PI)
            {
                yaw = yaw - M_PI * 2;
            }
            if (yaw <= -M_PI)
            {
                yaw = yaw + M_PI * 2;
            }
        }
    }
    // 获取位置
    void GetPos()
    {
        position[1] += step.second * sin(yaw_);
        position[0] += step.second * cos(yaw_);
    }
    // 
    // 脚步探测，输入的是合加速度
    bool StepDetection()
    {
        double interval = 0.35;
        bool isStatic = false;
        static bool peakStatic = false;
        static bool valleyStatic = false;
        static double zeroPeakTime = 0.0;
        static double zeroValleyTime = 0.0;
        static double zeroPeak = 0.0;
        static double zeroValley = 0.0;
        if (acc_norm_cur.second > acc_norm_pre.second && acc_norm_cur.second > acc_norm.front().second)
        {
            if (acc_norm_cur.first - peak.first > interval && acc_norm_cur.second > step_max)
            {
                peak = acc_norm_cur;
                isPeak = 1;
                yaw_peak = yaw_cur;
            }
            else if (acc_norm_cur.second < zero_max && acc_norm_cur.second >zero_min)
            {
                peakStatic = true;
                zeroPeakTime = acc_norm_cur.first;
                zeroPeak = acc_norm_cur.second;
            }
        }
        if (acc_norm_cur.second < acc_norm_pre.second && acc_norm_cur.second < acc_norm.front().second)
        {
            if (acc_norm_cur.first - valley.first > interval && acc_norm_cur.second < step_min)
            {
                valley = acc_norm_cur;
                isValley = 1;
                yaw_val = yaw_cur;
            }
            else if (acc_norm_cur.second > zero_min && acc_norm_cur.second < zero_max)
            {
                zeroValleyTime = acc_norm_cur.first;
                valleyStatic = true;
                zeroValley = acc_norm_cur.second;
            }
        }
        if (isValley && isPeak)
        {
            dt = abs(peak.first - valley.first);
            step.first = std::min(peak.first, valley.first);
            step.second = StepStride(peak.second, valley.second);
            yaw_ = (peak.first > valley.first) ? yaw_val : yaw_peak;
            isValley = 0;
            isPeak = 0;
            peakStatic = false;
            valleyStatic = false;
            savefile << setprecision(15) << peak.first << "\t" << peak.second << "\t" << valley.first << "\t" << valley.second << "\t"
                     << "1" << endl;
            return true;
        }
        // 零速判断
        if (peakStatic && valleyStatic)
        {
            dt = abs(zeroValleyTime - zeroPeakTime);
            step.first = std::min(zeroValleyTime, zeroPeakTime);
            step.second = 0;
            savefile << setprecision(15) << zeroPeakTime << "\t" << zeroPeak << "\t" << zeroValleyTime << "\t" << zeroValley << "\t"
                     << "0" << endl;
            peakStatic = false;
            valleyStatic = false;
            isValley = 0;
            isPeak = 0;
            return true;
        }
        savefile << setprecision(15) << acc_norm_cur.first << "\t" << acc_norm_cur.second << "\t" << acc_norm_cur.first << "\t" << acc_norm_cur.second << "\t"
                 << "2" << endl;
        return false;
    }
    // 64阶fir低通滤波器,给合加速度滤波
    void Filter(int idx)
    {
        std::vector<double> bl = {-0.0001966789842901, -0.0003100821779856, -0.0004351665007103, -0.0005492627467136,
                                  -0.0006019219794561, -0.0005223864918298, -0.0002372257688188, 0.0003043713018684,
                                  0.00110268625488, 0.002083838883137, 0.003087975981552, 0.003875866941265,
                                  0.004157803247822, 0.003644506074243, 0.002114664506824, -0.0005112945576257,
                                  -0.004105429273125, -0.008290255164899, -0.01243466910337, -0.01569714078084,
                                  -0.0171177313725, -0.01575105786208, -0.01082294652262, -0.001886210718675,
                                  0.01105242723235, 0.02746056031893, 0.04628271178162, 0.06602505905182,
                                  0.08491589395273, 0.1011214140548, 0.1129852733874, 0.1192544070343,
                                  0.1192544070343, 0.1129852733874, 0.1011214140548, 0.08491589395273,
                                  0.06602505905182, 0.04628271178162, 0.02746056031893, 0.01105242723235,
                                  -0.001886210718675, -0.01082294652262, -0.01575105786208, -0.0171177313725,
                                  -0.01569714078084, -0.01243466910337, -0.008290255164899, -0.004105429273125,
                                  -0.0005112945576257, 0.002114664506824, 0.003644506074243, 0.004157803247822,
                                  0.003875866941265, 0.003087975981552, 0.002083838883137, 0.00110268625488,
                                  0.0003043713018684, -0.0002372257688188, -0.0005223864918298, -0.0006019219794561,
                                  -0.0005492627467136, -0.0004351665007103, -0.0003100821779856, -0.0001966789842901};

        double lpf = 0;
        for (int i = 0; i < 64; i++)
        {
            lpf += acc_norm[idx + i].second * bl[i];
        }
        acc_norm[idx].second = lpf;
    }
    // 求峰值，要多统计两个数据
    bool PDRCore(double time, Eigen::Vector3d gry, Eigen::Vector3d acc)
    {
        // gry = gry * D2R;
        if (!pdr_inited)
        {
            peak.first = -INT16_MIN;
            valley.first = -INT16_MIN;
            lastTime = time;
            pdr_inited = 1;
            return false;
        }

        double an = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]) - g;
        CalYaw(gry, acc, attitude, time - lastTime);
        lastTime = time;
        std::pair<double, double> temp(time, an);
        if (acc_norm.size() < 64)
        {
            acc_norm.push_back(temp);
            if (acc_norm.size() == 64)
            {
                Filter(0);
                acc_norm_pre = acc_norm.front();
            }
            return false;
        }
        acc_norm.pop_front();
        acc_norm.push_back(temp);
        Filter(0);
        if (pdr_inited < 2)
        {
            acc_norm_cur = acc_norm.front();
            yaw_cur = attitude[2];
            pdr_inited++;
            return false;
        }
        if (StepDetection())
        {
            GetPos();
            // savefile << position[0] << "\t" << position[1] << "\t" << attitude[2] << endl;
            return true;
        }
        acc_norm_pre = acc_norm_cur;
        acc_norm_cur = acc_norm.front();
        yaw_cur = attitude[2];
        return false;
    }
    void GetOdometry(nav_msgs::Odometry &odo)
    {
        // odo.header.stamp = ros::Time(step.first);
        odo.pose.pose.position.x = position[0];
        odo.pose.pose.position.y = position[1];
        tf::Quaternion q;
        q.setRPY(attitude[0], attitude[1], attitude[2]);
        odo.pose.pose.orientation.x = q.x();
        odo.pose.pose.orientation.y = q.y();
        odo.pose.pose.orientation.z = q.z();
        odo.pose.pose.orientation.w = q.w();
        // odo.pose.pose.orientation.
    }
    
    void GetPosInB(double &pos){
        pos = sqrt(position[0] * position[0] + position[1] * position[1]);
    }
private:
    double dt = 0;
    double lastTime;                                // 上一时刻时间
    std::deque<std::pair<double, double>> acc_norm; // pair 时间和去除重力加速度合加速度
    std::pair<double, double> acc_norm_pre;         // 前一个合加速度
    std::pair<double, double> acc_norm_cur;         // 当前合加速度
    std::pair<double, double> peak;                 // 峰值
    std::pair<double, double> valley;               // 谷值
    std::pair<double, double> step;                 // 脚步的 时间和步长
    double step_max;                        // 最大值的阈值
    double step_min;                        // 最小值的阈值
    double zero_max;                                //零速判断的最大值
    double zero_min;                                //零速判断最小值
    Eigen::Vector3d position;                       // 位置
    Eigen::Vector3d attitude;                       // 姿态
    double yaw_;                                    // 每一个脚步的yaw
    double yaw_cur;                                 // 当前时刻的yaw
    double yaw_peak;                                // 暂时记录peak yaw
    double yaw_val;                                 // 暂时记录valley yaw
    int isPeak = 0;
    int isValley = 0;
    int pdr_inited; // 初始化
    std::ofstream savefile;
    double g; //当地重力加速度
};
#endif
