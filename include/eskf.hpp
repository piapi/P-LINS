//
// Created by Liaozongbo on 2023/12/10.
//

#ifndef ESKF_HPP
#define ESKF_HPP

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "imu2pdr.hpp"
#include "iostream"
#include <glog/logging.h>
#include <iomanip>
#include "common/common_lib.h"
#include "sensor_msgs/Imu.h"
#include "common/point_type.h"
const bool time_list(PointXYZIRT &x, PointXYZIRT &y) { return (x.time < y.time); };

/**

 * 可以指定观测GNSS的读数，GNSS应该事先转换到车体坐标系
 *
 * 使用18维的ESKF，标量类型可以由S指定，默认取double
 * 变量顺序：p, v, R, bg, ba, grav，
 * @tparam S    状态变量的精度，取float或double
 */
template <typename S = double>
class ESKF
{
public:
    /// 类型定义
    using SO3 = Sophus::SO3<S>;                    // 旋转变量类型
    using VecT = Eigen::Matrix<S, 3, 1>;           // 向量类型
    using Vec18T = Eigen::Matrix<S, 18, 1>;        // 18维向量类型
    using Mat3T = Eigen::Matrix<S, 3, 3>;          // 3x3矩阵类型
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>; // 运动噪声类型
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;     // 里程计噪声类型
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;     // GNSS噪声类型
    using Mat18T = Eigen::Matrix<S, 18, 18>;       // 18维方差类型
    using NavStateT = NavState<S>;                 // 整体名义状态变量类型
    struct Options
    {
        Options() = default;

        /// IMU 测量与零偏参数
        double imu_dt_ = 0.01; // IMU测量间隔
        // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
        VecT gyro_var_{1e-5, 1e-5, 1e-5};     // 陀螺测量标准差
        VecT acce_var_{1e-2,1e-2,1e-2};      // 加计测量标准差
        double bias_gyro_var_ = 1e-6; // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4; // 加计零偏游走标准差

        /// 里程计参数
        double odom_var_ = 0.5;
        double odom_span_ = 0.1;       // 里程计测量间隔
        double wheel_radius_ = 0.155;  // 轮子半径
        double circle_pulse_ = 1024.0; // 编码器每圈脉冲数

        //

        /// RTK 观测参数
        double gnss_pos_noise_ = 0.1;                  // GNSS位置噪声
        double gnss_height_noise_ = 0.1;               // GNSS高度噪声
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD; // GNSS旋转噪声

        /// 其他配置
        bool update_bias_gyro_ = true; // 是否更新陀螺bias
        bool update_bias_acce_ = true; // 是否更新加计bias
    };
 struct Pose6D {
    float offset_time;           // 时间偏移
    Eigen::Vector3d acc;         // 加速度
    Eigen::Vector3d gyr;         // 陀螺仪数据
    Eigen::Vector3d vel;         // 速度
    Eigen::Vector3d pos;         // 位置
    Eigen::Matrix3d rot;         // 旋转（可能需要更改为适当的表示方式）

    // 构造函数
    Pose6D(float offset, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& gyroscope,
           const Eigen::Vector3d& velocity, const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation)
        : offset_time(offset), acc(acceleration), gyr(gyroscope),
          vel(velocity), pos(position), rot(rotation) {}
};
    /**
     * 初始零偏取零
     */
    ESKF(Options option = Options()) : options_(option)
    {
        BuildNoise(option);
    }

    /**
     * 设置初始条件
     * @param options 噪声项配置
     * @param init_bg 初始零偏 陀螺
     * @param init_ba 初始零偏 加计
     * @param gravity 重力
     */
    void SetInitialConditions(Options options, const VecT &init_bg, const VecT &init_ba,
                              const VecT &gravity = VecT(0, 0, -9.8))
    {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
        cov_ = Mat18T::Identity() * 1e-4;
    }
    void SetIMU(const sensor_msgs::Imu &imu_in)
    {
        last_imu_ = imu_in;
        current_time_ = last_imu_.header.stamp.toSec();
    }
    void SetPVQ(const Mat3T &Rot, const VecT &pos = VecT::Zero(), const VecT &vel = VecT::Zero())
    {
        p_ = pos;
        v_ = vel;
        R_ = SO3(Rot);
    }
    /// 使用IMU递推
    bool Predict(const sensor_msgs::Imu &imu_in);
    void UndistortPcl(MeasureGroup &measures, pcl::PointCloud<PointXYZIRT>::Ptr &pcl_out);

    /// 使用轮速计观测
    // bool ObserveWheelSpeed();

    /// 使用PDR观测
    bool ObservePDRSpeed(const Vec3d &velocity);

    //
    bool ObserveNHC();

    /// 使用GPS观测
    // bool ObserveGps(const GNSS& gnss);

    /**
     * 使用SE3进行观测
     * @param pose  观测位姿
     * @param trans_noise 平移噪声
     * @param ang_noise   角度噪声
     * @return
     */
    bool ObserveSE3(const SE3 &pose, double trans_noise = 0.01, double ang_noise = 0.1 * math::kDEG2RAD);
    bool ObserveSE3(const Vec3d &pos, const Mat3d &Rot, double trans_noise = 0.01, double ang_noise = 0.1 * math::kDEG2RAD);

    /// 获取SE3 状态
    SE3 GetNominalSE3() const { return SE3(R_, p_); }
    // 获取P阵
    Mat18T GetP(){
        return cov_;
    }
    NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

    /// 设置状态X
    void SetX(const NavStated &x, const Vec3d &grav)
    {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        g_ = grav;
    }

    /// 设置协方差
    void SetCov(const Mat18T &cov) { cov_ = cov; }

    /// 获取重力
    Vec3d GetGravity() const { return g_; }

private:
    void BuildNoise(const Options &options)
    {
        double ev = options.acce_var_[0];
        double et = options.gyro_var_[0];
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev; // * ev;
        double et2 = et; // * et;
        double eg2 = eg; // * eg;
        double ea2 = ea; // * ea;

        // 设置过程噪声
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

        // 设置里程计噪声
        double o2 = options_.odom_var_ * options_.odom_var_;
        odom_noise_.diagonal() << o2, o2, o2;

        // 设置GNSS状态
        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
    }

    /// 更新名义状态变量，重置error state
    void UpdateAndReset()
    {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (options_.update_bias_gyro_)
        {
            bg_ += dx_.template block<3, 1>(9, 0);
        }

        if (options_.update_bias_acce_)
        {
            ba_ += dx_.template block<3, 1>(12, 0);
        }

        g_ += dx_.template block<3, 1>(15, 0);

        ProjectCov();
        dx_.setZero();
    }

    /// 对P阵进行投影，参考式(3.63)
    void ProjectCov()
    {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

    /// 成员变量
    double current_time_ = 0.0; // 当前时间
    sensor_msgs::Imu last_imu_; // 上时刻的IMU数据
    Eigen::Vector3d angvel_last = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_s_last = Eigen::Vector3d::Zero();
    /// 名义状态
    VecT p_ = VecT::Zero();
    VecT v_ = VecT::Zero();
    SO3 R_;
    VecT bg_ = VecT::Zero();
    VecT ba_ = VecT::Zero();
    VecT g_{0, 0, -9.8};
    std::vector<Pose6D> IMUpose;

    /// 误差状态
    Vec18T dx_ = Vec18T::Zero();

    /// 协方差阵
    Mat18T cov_ = Mat18T::Identity();

    /// 噪声阵
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

    /// 标志位
    bool first_gnss_ = true; // 是否为第一个gnss数据

    /// 配置项
    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
void ESKF<S>::UndistortPcl(MeasureGroup &measures, pcl::PointCloud<PointXYZIRT>::Ptr &pcl_out){
 /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = measures.imu;
  v_imu.push_front(last_imu_);
  last_imu_ = v_imu.back();
  const double &imu_beg_time = v_imu.front().header.stamp.toSec();
  const double &imu_end_time = v_imu.back().header.stamp.toSec();
  const double &pcl_beg_time = measures.lidar_beg_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = measures.lidar;
  std::sort(pcl_out->points.begin(), pcl_out->points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_out->points.back().time;
  std::cout << "[ IMU Process ]: Process lidar from " << std::setprecision(15) << pcl_beg_time << " to " << pcl_end_time << ", "
            << measures.imu.size() << " imu msgs from " << imu_beg_time << " to " << imu_end_time << std::endl;

  /*** Initialize IMU pose ***/
  IMUpose.clear();
  // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end, state.pos_end, state.rot_end));
  IMUpose.push_back(Pose6D(0.0, acc_s_last, angvel_last, v_, p_, R_.matrix()));

  /*** forward propagation at each imu point ***/
  Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(v_), pos_imu(p_);
  Eigen::Matrix3d R_imu(R_.matrix());
  Eigen::MatrixXd F_x = Mat18T::Identity();
  Eigen::MatrixXd cov_w=Mat18T::Zero();
  double dt = 0;
  for (int it_imu = 0; it_imu < v_imu.size() - 1; it_imu++)
  {
      auto &&head = v_imu[it_imu];
      auto &&tail = v_imu[it_imu + 1];

      angvel_avr << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
          0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
          0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
      acc_avr << 0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
          0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
          0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);

      angvel_avr -= bg_;
      acc_avr = acc_avr * 9.8 - ba_;

      dt = tail.header.stamp.toSec() - head.header.stamp.toSec();

      /* covariance propagation */
      Eigen::Matrix3d acc_avr_skew;
      Eigen::Matrix3d Exp_f = SO3::exp(angvel_avr * dt).matrix();
      acc_avr_skew << SKEW_SYM_MATRX(angvel_avr);
                                             // 主对角线
      F_x. block<3, 3>(0, 3) = Eye3d * dt;                      // p 对 v
      F_x. block<3, 3>(3, 6) = -R_imu * acc_avr_skew * dt; // v对theta
      F_x. block<3, 3>(3, 12) = -R_imu * dt;;                          // v 对 ba
      F_x. block<3, 3>(3, 15) = Eye3d * dt;                     // v 对 g
      F_x. block<3, 3>(6, 6) = -Exp_f;    // theta 对 theta
      F_x. block<3, 3>(6, 9) = -Eye3d * dt;                     // theta 对 bg

      Eigen::Matrix3d cov_acc_diag(Eye3d), cov_gyr_diag(Eye3d);
      cov_acc_diag.diagonal() = options_.acce_var_;
      cov_gyr_diag.diagonal() = options_.gyro_var_;
      cov_w.block<3, 3>(0, 0) = cov_gyr_diag * dt * dt * 10000;
      cov_w.block<3, 3>(3, 3) = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt * 10000;
      cov_w.block<3, 3>(6, 6) = R_imu * cov_acc_diag * R_imu.transpose() * dt * dt * 10000;
      cov_w.block<3, 3>(9, 9) = Eye3d * 0.0001 * dt * dt;   // bias gyro covariance
      cov_w.block<3, 3>(12, 12) = Eye3d * 0.0001 * dt * dt; // bias acc covariance

      cov_ = F_x * cov_ * F_x.transpose() + cov_w;

      /* propogation of IMU attitude */
      R_imu = R_imu * Exp_f;

      /* Specific acceleration (global frame) of IMU */
      acc_imu = R_imu * acc_avr + g_;

      /* propogation of IMU */
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

      /* velocity of IMU */
      vel_imu = vel_imu + acc_imu * dt;

      /* save the poses at each IMU measurements */
      angvel_last = angvel_avr;
      acc_s_last = acc_imu;
      double &&offs_t = tail.header.stamp.toSec() - pcl_beg_time;
      // std::cout<<"acc "<<acc_imu.transpose()<<"vel "<<acc_imu.transpose()<<"vel "<<pos_imu.transpose()<<std::endl;
      IMUpose.push_back(Pose6D(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  dt = pcl_end_time - imu_end_time;
  v_ = vel_imu + acc_imu * dt;
  R_ = SO3(R_imu) * SO3::exp(angvel_avr * dt);
  p_ = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

  Eigen::Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284);
  auto pos_liD_e = p_ + R_.matrix() * Lidar_offset_to_IMU;
  // auto R_liD_e   = state_inout.rot_end * Lidar_R_to_IMU;


  /*** undistort each lidar point (backward propagation) ***/
  auto it_pcl = pcl_out->points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu = head->rot;
      acc_imu = head->acc;
      // std::cout<<"head imu acc: "<<acc_imu.transpose()<<std::endl;
      vel_imu = head->vel;
      pos_imu = head->pos;
      angvel_avr = head->gyr;

      int i = 0;
      for (; it_pcl->time > head->offset_time; it_pcl--)
      {
          dt = it_pcl->time - head->offset_time;

          /* Transform to the 'end' frame, using only the rotation
           * Note: Compensation direction is INVERSE of Frame's moving direction
           * So if we want to compensate a point at timestamp-i to the frame-e
           * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
          Eigen::Matrix3d R_i(R_imu * SO3::exp(angvel_avr * dt).matrix());
          Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt + R_i * Lidar_offset_to_IMU - pos_liD_e);

          Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
          Eigen::Vector3d P_compensate = R_.matrix().transpose() * (R_i * P_i + T_ei);

          /// save Undistorted points and their rotation
          it_pcl->x = P_compensate(0);
          it_pcl->y = P_compensate(1);
          it_pcl->z = P_compensate(2);

          if (it_pcl == pcl_out->points.begin())
              break;
      }
  }
  
}
template <typename S>
bool ESKF<S>::Predict(const sensor_msgs::Imu &imu_in)
{
    double timestamp = imu_in.header.stamp.toSec();
    assert(timestamp >= current_time_);

    double dt = timestamp - current_time_;
    if (dt > (50 * options_.imu_dt_) || dt < 0)
    {
        // 时间间隔不对，可能是第一个IMU数据，没有历史信息
        ROS_WARN("skip this imu_in because dt_ = %.2f", dt);
        current_time_ = timestamp;
        return false;
    }
    // std::cout << dt << std::endl;
    // nominal state 递推
    Eigen::Vector3d acc_cur(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    Eigen::Vector3d gyr_cur(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    Eigen::Vector3d acc_prev(last_imu_.linear_acceleration.x, last_imu_.linear_acceleration.y, last_imu_.linear_acceleration.z);
    Eigen::Vector3d gyr_prev(last_imu_.angular_velocity.x, last_imu_.angular_velocity.y, last_imu_.angular_velocity.z);

    VecT w_kplus1 = 0.5 * (gyr_prev + gyr_cur) - bg_;
    VecT d_theta = w_kplus1 * dt;
    SO3 new_R = R_ * SO3::exp(d_theta);
    VecT acc_w = 0.5 * (new_R * (acc_cur - ba_) + R_ * (acc_prev - ba_)) + g_;
    VecT new_v = v_ + acc_w * dt;
    VecT new_p = p_ + v_ * dt + 0.5 * acc_w * dt * dt;


    R_ = new_R;
    v_ = new_v;
    p_ = new_p;
    // 其余状态维度不变

    // error state 递推
    // 计算运动过程雅可比矩阵 F，见(3.47)
    // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式，这里为了教学方便，使用矩阵形式
    Mat18T F = Mat18T::Identity();                                              // 主对角线
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                      // p 对 v
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(acc_cur - ba_) * dt; // v对theta
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                          // v 对 ba
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                     // v 对 g
    F.template block<3, 3>(6, 6) = SO3::exp(-(gyr_cur - bg_) * dt).matrix();    // theta 对 theta
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                     // theta 对 bg

    // mean and cov prediction
    dx_ = F * dx_; // 这行其实没必要算，dx_在重置之后应该为零，因此这步可以跳过，但F需要参与Cov部分计算，所以保留
    cov_ = F * cov_.eval() * F.transpose() + Q_;
    current_time_ = timestamp;
    last_imu_ = imu_in;
    return true;
}

// template <typename S>
// bool ESKF<S>::ObserveWheelSpeed(const Odom& odom) {
//     assert(odom.timestamp_ >= current_time_);
//     // odom 修正以及雅可比
//     // 使用三维的轮速观测，H为3x18，大部分为零
//     Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
//     H.template block<3, 3>(0, 3) = Mat3T::Identity();

//     // 卡尔曼增益
//     Eigen::Matrix<S, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();

//     // velocity obs
//     double velo_l = options_.wheel_radius_ * odom.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
//     double velo_r =
//         options_.wheel_radius_ * odom.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
//     double average_vel = 0.5 * (velo_l + velo_r);

//     VecT vel_odom(average_vel, 0.0, 0.0);
//     VecT vel_world = R_ * vel_odom;

//     dx_ = K * (vel_world - v_);

//     // update cov
//     cov_ = (Mat18T::Identity() - K * H) * cov_;

//     UpdateAndReset();
//     return true;
// }
template <typename S>

bool ESKF<S>::ObservePDRSpeed(const Vec3d &velocity)
{
    // odom 修正以及雅可比
    // 使用三维的轮速观测，H为3x18，大部分为零
    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    H.template block<3, 3>(0, 3) = Mat3T::Identity();

    // 卡尔曼增益
    Eigen::Matrix<S, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();

    VecT vel_world = R_ * velocity;

    dx_ = K * (vel_world - v_);

    // update cov
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}
// template <typename S>
// bool ESKF<S>::ObserveGps(const GNSS& gnss) {
//     /// GNSS 观测的修正
//     assert(gnss.unix_time_ >= current_time_);

//     if (first_gnss_) {
//         R_ = gnss.utm_pose_.so3();
//         p_ = gnss.utm_pose_.translation();
//         first_gnss_ = false;
//         current_time_ = gnss.unix_time_;
//         return true;
//     }

//     assert(gnss.heading_valid_);
//     ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise_, options_.gnss_ang_noise_);
//     current_time_ = gnss.unix_time_;

//     return true;
// }

template <typename S>

bool ESKF<S>::ObserveNHC()
{
    // odom 修正以及雅可比
    // 使用三维的轮速观测，H为3x18，大部分为零
    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    H.template block<3, 3>(0, 3) = Mat3T::Identity();
    Vec3d velocity(v_.x(), 0, 0);
    // 卡尔曼增益
    Eigen::Matrix<S, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();

    VecT vel_world = R_ * velocity;

    dx_ = K * (vel_world - v_);

    // update cov
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3 &pose, double trans_noise, double ang_noise)
{
    /// 既有旋转，也有平移
    /// 观测状态变量中的p, R，H为6x18，其余为零
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity(); // P部分
    H.template block<3, 3>(3, 6) = Mat3T::Identity(); // R部分（3.66)

    // 卡尔曼增益和更新过程
    Vec6d noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;

    Mat6d V = noise_vec.asDiagonal();
    Eigen::Matrix<S, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

    // 更新x和cov
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_);         // 平移部分
    innov.template tail<3>() = (R_.inverse() * pose.so3()).log(); // 旋转部分(3.67)

    dx_ = K * innov;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}
template <typename S>
bool ESKF<S>::ObserveSE3(const Vec3d &pos, const Mat3d &Rot, double trans_noise, double ang_noise)
{
    SE3 pose(Rot, pos);
    return ObserveSE3(pose, trans_noise, ang_noise);
}

// namespace sad

#endif // SLAM_IN_AUTO_DRIVING_ESKF_HPP
