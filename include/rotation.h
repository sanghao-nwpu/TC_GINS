#ifndef ROTATION_H
#define ROTATION_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation {

public:
    static Quaterniond matrix2quaternion(const Matrix3d &matrix) {
        return Quaterniond(matrix);
    }

    static Matrix3d quaternion2matrix(const Quaterniond &quaternion) {
        return quaternion.toRotationMatrix();
    }

    // ZYX旋转顺序, 前右下的IMU, 输出RPY
    static Vector3d matrix2euler(const Eigen::Matrix3d &dcm) {
        Vector3d euler;

        euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

        if (dcm(2, 0) <= -0.999) {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
        } else if (dcm(2, 0) >= 0.999) {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
        } else {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = atan2(dcm(1, 0), dcm(0, 0));
        }

        // heading 0~2PI
        if (euler[2] < 0) {
            euler[2] = M_PI * 2 + euler[2];
        }

        return euler;
    }

    static Vector3d quaternion2euler(const Quaterniond &quaternion) {
        return matrix2euler(quaternion.toRotationMatrix());
    }

    static Quaterniond rotvec2quaternion(const Vector3d &rotvec) {
        double angle = rotvec.norm();
        Vector3d vec = rotvec.normalized();
        return Quaterniond(Eigen::AngleAxisd(angle, vec));
    }

    static Vector3d quaternion2vector(const Quaterniond &quaternion) {
        Eigen::AngleAxisd axisd(quaternion);
        return axisd.angle() * axisd.axis();
    }

    // RPY --> C_b^n, ZYX顺序
    static Matrix3d euler2matrix(const Vector3d &euler) {
        return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                        Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    static Quaterniond euler2quaternion(const Vector3d &euler) {
        return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    static Matrix3d skewSymmetric(const Vector3d &vector) {
        Matrix3d mat;
        mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
        return mat;
    }

    static Eigen::Matrix4d quaternionleft(const Quaterniond &q) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = q.w();
        ans.block<1, 3>(0, 1) = -q.vec().transpose();
        ans.block<3, 1>(1, 0) = q.vec();
        ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + skewSymmetric(q.vec());
        return ans;
    }

    static Eigen::Matrix4d quaternionright(const Quaterniond &p) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = p.w();
        ans.block<1, 3>(0, 1) = -p.vec().transpose();
        ans.block<3, 1>(1, 0) = p.vec();
        ans.block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - skewSymmetric(p.vec());
        return ans;
    }

    static Eigen::Matrix3d Jleft(const Eigen::AngleAxisd &so3)
    {
        double p = so3.angle();

        if (p < 1e-3) {
            return Eigen::Matrix3d::Identity();
        }
        double sp = sin(p);
        double cp = cos(p);

        Eigen::Vector3d axis = so3.axis();
        Eigen::Matrix3d Jl = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Jl = sp / p * I + (1 - sp) / p * axis * axis.transpose() + (1 - cp) / p * skewSymmetric(axis);
        return Jl;
    }

    static Eigen::Matrix3d Jright(const Eigen::AngleAxisd &so3)
    {
        double theta = so3.angle();
        Eigen::Vector3d axis = so3.axis();
        Eigen::AngleAxisd so3_inv(theta, -axis);
        /** 直接调用Jleft即可 */
        return Jleft(so3_inv);
    }
};

#endif // ROTATION_H