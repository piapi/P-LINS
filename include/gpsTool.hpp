
#ifndef GPSTOOL_HPP_
#define GPSTOOL_HPP_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <Eigen/Core>
#define DEG_TO_RAD 0.01745329252
#define EARTH_A 6378137.0            ///< WGS84 长半径a
#define EARTH_B 6356752.31424518    ///< WGS84 短半径b

class GpsTools {
 public:
  GpsTools() { lla_origin_.setIdentity(); }

  static Eigen::Vector3d GpsMsg2Eigen(const sensor_msgs::NavSatFix &gps_msgs) {
    Eigen::Vector3d
        lla(gps_msgs.latitude, gps_msgs.longitude, gps_msgs.altitude);
    return lla;
  }



  Eigen::Vector3d LLA2ECEF(const Eigen::Vector3d &lla) {
    Eigen::Vector3d ecef;
    double lat = deg2rad(lla[0]);
    double lon = deg2rad(lla[1]);
    double alt = lla[2];
    double N = pow(EARTH_A, 2)
        / sqrt(pow(EARTH_A * cos(lat), 2) + pow(EARTH_B * sin(lat), 2));
    ecef[0] = (N + alt) * cos(lat) * cos(lon);
    ecef[1] = (N + alt) * cos(lat) * sin(lon);
    ecef[2] = (pow(EARTH_B / EARTH_A, 2) * N + alt) * sin(lat);

    return ecef;
  }

  Eigen::Vector3d ECEF2LLA(const Eigen::Vector3d &ecef) {
    double e =
        sqrt((pow(EARTH_A, 2) - pow(EARTH_B, 2)) / pow(EARTH_A, 2));
    double e_ =
        sqrt((pow(EARTH_A, 2) - pow(EARTH_B, 2)) / pow(EARTH_B, 2));
    double p = sqrt(pow(ecef[0], 2) + pow(ecef[1], 2));
    double theta = atan2(ecef[2] * EARTH_A, p * EARTH_B);

    double lon = atan2(ecef[1], ecef[0]);
    double lat = atan2((ecef[2] + pow(e_, 2) * EARTH_B * pow(sin(theta), 3)),
                       p - pow(e, 2) * EARTH_A * pow(cos(theta), 3));
    double N = pow(EARTH_A, 2)
        / sqrt(pow(EARTH_A * cos(lat), 2) + pow(EARTH_B * sin(lat), 2));
    double alt = p / cos(lat) - N;
    Eigen::Vector3d lla(rad2deg(lat), rad2deg(lon), alt);
    return lla;
  }

  Eigen::Vector3d ECEF2ENU(const Eigen::Vector3d &ecef) {
    double lat = deg2rad(lla_origin_[0]);
    double lon = deg2rad(lla_origin_[1]);

    Eigen::Vector3d t = -LLA2ECEF(lla_origin_);
    Eigen::Matrix3d r;
    r << -sin(lon), cos(lon), 0,
        -cos(lon) * sin(lat), -sin(lat) * sin(lon), cos(lat),
        cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat);

    Eigen::Vector3d enu;
    enu = ecef + t;
    enu = r * enu;
    return enu;
  }

  Eigen::Vector3d ENU2ECEF(const Eigen::Vector3d &enu) {
    double lat = deg2rad(lla_origin_[0]);
    double lon = deg2rad(lla_origin_[1]);

    Eigen::Vector3d t = LLA2ECEF(lla_origin_);
    Eigen::Matrix3d r;
    r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat),
        cos(lon), -sin(lon) * sin(lat), sin(lon) * cos(lat),
        0, cos(lat), sin(lat);
    Eigen::Vector3d ecef;
    ecef = r * enu + t;
    return ecef;
  }

  void updateGPSpose(const sensor_msgs::NavSatFix &gps_msgs) {
    //检查状态4
    if (gps_msgs.status.status == 4 || gps_msgs.status.status == 5 || gps_msgs.status.status == 1
        || gps_msgs.status.status == 2) {
      //第一个的时候设置为起点
      if (lla_origin_ == Eigen::Vector3d::Identity()) {
        Eigen::Vector3d lla = GpsMsg2Eigen(gps_msgs);
        lla_origin_ = lla;
        std::cout << "GPS origin: " << lla_origin_ << "\n status: " << gps_msgs.status.status << std::endl;
      } else {
        Eigen::Vector3d lla = GpsMsg2Eigen(gps_msgs);
        Eigen::Vector3d ecef = LLA2ECEF(lla);
        Eigen::Vector3d enu = ECEF2ENU(ecef);
        gps_pos_ = enu;
        std::cout << "GPS lla_origin_: " << lla_origin_ << "\n curr" << gps_pos_ << std::endl;
      }
    }
  }

//变量部分
  //1.lla的起点
  Eigen::Vector3d lla_origin_;
  //2.enu下的坐标
  Eigen::Vector3d gps_pos_;
 private:
  static inline double deg2rad(const double &deg) {
    return deg * DEG_TO_RAD;
  }
  static inline double rad2deg(const double &rad) {
    return rad / DEG_TO_RAD;
  }

};

#endif 
