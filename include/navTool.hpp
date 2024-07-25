#include <ros/ros.h>
#include <Eigen/Dense>
#include <deque>
class NavTool
{
public:
    /*
     * @Description: 地球模型
     */
    double a = 6378137.0;
    double b = 6356752.3142451793;
    double f = 0.0033528106647474805;
    double w_e = 7.2921151467E-5;
    double e2 = 0.0066943799901413156;
    double GM = 398600441800000.00;

public:
    double NavTool::GetRm(double latitude)
    {
        return a * (1 - e2) / pow((1 - e2 * sin(latitude) * sin(latitude)), 1.5);
    }
    double NavTool::GetRn(double latitude)
    {
        return a / sqrt(1 - e2 * sin(latitude) * sin(latitude));
    }
    Eigen::Vector3d NavTool::GetW_ie(double latitude)
    {
        Eigen::Vector3d wie;
        wie << w_e * cos(latitude), 0, -w_e * sin(latitude);
        return wie;
    }
    Eigen::Vector3d NavTool::GetW_en(Eigen::Vector3d position, Eigen::Vector3d velocity, double Rm, double Rn)
    {
        Eigen::Vector3d wen;
        wen << velocity[1] / (Rn + position[2]),
            -velocity[0] / (Rm + position[2]),
            -velocity[1] * tan(position[0]) / (Rn + position[2]);
        return wen;
    }
    Eigen::Quaterniond NavTool::Pos2Qne(Eigen::Vector3d position)
    {
        double lat = position[0];
        double lon = position[1];
        double s1 = sin(lon / 2);
        double c1 = cos(lon / 2);
        double s2 = sin(-M_PI / 4 - lat / 2);
        double c2 = cos(-M_PI / 4 - lat / 2);
        return Eigen::Quaterniond(c1 * c2, -s1 * s2, c1 * s2, c2 * s1);
    }
    void NavTool::Quatne2Pos(Eigen::Quaterniond q_ne, Eigen::Vector3d &position)
    {
        double lat = -2 * atan(q_ne.y() / q_ne.w()) - M_PI / 2;
        double lon = 2 * atan2(q_ne.z(), q_ne.w());

        position[0] = lat;
        position[1] = lon;
    }
    Eigen::Vector3d NavTool::Dpos2Rvec(double latitude, double delta_lat, double delta_lon)
    {
        Eigen::Vector3d rev;
        rev << delta_lon * cos(latitude),
            -delta_lat,
            -delta_lon * sin(latitude);
        return rev;
    }
    Eigen::Quaterniond NavTool::Rvec2Quat(Eigen::Vector3d rotation_vector)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(1, 0, 0, 0);
        double mag2 = rotation_vector.dot(rotation_vector.transpose());
        if (mag2 < M_PI * M_PI)
        {
            mag2 = 0.25 * mag2;
            double c = 1.0 - mag2 / 2.0 * (1.0 - mag2 / 12.0 * (1.0 - mag2 / 30.0));
            double s = 1.0 - mag2 / 6.0 * (1.0 - mag2 / 20.0 * (1.0 - mag2 / 42.0));
            q = Eigen::Quaterniond(c, s * 0.5 * rotation_vector[0], s * 0.5 * rotation_vector[1],
                                   s * 0.5 * rotation_vector[2]);
        }
        else
        {
            double mag = sqrt(mag2);
            double s_mag = sin(mag / 2);

            q = Eigen::Quaterniond(
                cos(mag / 2.0), rotation_vector[0] * s_mag / mag, rotation_vector[1] * s_mag / mag,
                rotation_vector[2] * s_mag / mag);

            if (q.w() < 0)
                q.coeffs() = -q.coeffs();
        }
        return q;
    }
    Eigen::Vector3d NavTool::NormalGravity(Eigen::Vector3d position)
    {
        double a1 = 9.7803267715;
        double a2 = 0.0052790414;
        double a3 = 0.0000232718;
        double a4 = -0.000003087691089;
        double a5 = 0.000000004397731;
        double a6 = 0.000000000000721;
        double s2 = sin(position[0]) * sin(position[0]);
        double s4 = s2 * s2;
        return Eigen::Vector3d(0, 0,
                               a1 * (1 + a2 * s2 + a3 * s4) + (a4 + a5 * s2) * position[2] + a6 * pow(position[2], 2));
    }
    Eigen::Matrix3d NavTool::Cp_form(Eigen::Vector3d vector3d)
    {
        Eigen::Matrix3d cp;
        cp << 0, -vector3d[2], vector3d[1],
            vector3d[2], 0, -vector3d[0],
            -vector3d[1], vector3d[0], 0;
        return cp;
    }
    double NavTool::dist_ang(double ang1, double ang2)
    {
        double ang = ang2 - ang1;
        return (ang > M_PI) ? ang - 2 * M_PI : (ang < -M_PI) ? ang + 2 * M_PI
                                                             : ang;
    }
    Eigen::Quaterniond NavTool::Euler2Quat(Eigen::Vector3d euler)
    {
        Eigen::AngleAxisd roll(euler[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(euler[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(euler[2], Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quater;
        quater = (yaw * pitch * roll).normalized();
        return quater;
    }
    Eigen::Vector3d NavTool::Quat2Euler(Eigen::Quaterniond quater)
    {
        Eigen::Vector3d eulerAngle = quater.toRotationMatrix().eulerAngles(0, 1, 2);
        return eulerAngle;
    }
};