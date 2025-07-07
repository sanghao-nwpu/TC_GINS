

#include "rotation.h"
#include "preintegration.h"

void Preintegration::reintergration()
{
    // TODO: implement reintergration
    if (imu_array_.size() < 2)
    {
        return ;
    }
    Mat3d I = Mat3d::Identity();
    Mat3d dR = Mat3d::Identity();
    Mat3d dRik = Mat3d::Identity();
    Vec3d dvik = Vec3d::Zero();
    Mat<double, 9, 9> A = Mat<double, 9, 9>::Zero();
    Mat<double, 9, 6> B = Mat<double, 9, 6>::Zero();
    Mat<double, 6, 6> Q = Mat<double, 6, 6>::Zero();
    
    double dt = 0.0;

    for (size_t i = 0; i < imu_array_.size() - 1; i++)
    {
        dt = imu_array_[i+1].timestamp() - imu_array_[i].timestamp();
        if (dt <= 0)
        {
            throw std::runtime_error("Invalid IMU timestamp sequence.");
        }
        /** IMU 去偏置, 直接调用类中的方法 */
        Vec3d gyro = imu_array_[i].gyro(I, bg_);
        Vec3d accel = imu_array_[i].accel(I, ba_);
        
        /** 状态积分 */
        dR = Quatd(gyro * dt).toRotationMatrix();
        dRik = delta_rot_;
        dvik = delta_vel_;

        delta_rot_ *= dR;
        delta_vel_ += dRik * accel * dt;
        delta_pos_ += dvik * dt + 0.5 * dRik * accel * dt * dt;

        /** 协方差更新 @ref gaoxiang */
        A.block<3, 3>(0, 0) = dR;
        A.block<3, 3>(3, 0) = -dR * Rotation::skewSymmetric(accel) * dt;
        A.block<3, 3>(3, 3) = Mat3d::Identity();
        A.block<3, 3>(6, 0) = -0.5 * dR * Rotation::skewSymmetric(accel) * dt * dt; 
        A.block<3, 3>(6, 3) = Mat3d::Identity() * dt;

        B.block<3, 3>(0, 0) = dR * dt;   /** TODO: Jr * dt */
        B.block<3, 3>(3, 3) = dR * dt;
        B.block<3, 3>(6, 3) = 0.5 * dR * dt * dt;

        cov_ = A * cov_ * A.transpose() + B * cov_ * B.transpose();
    }

    /** 记录积分时长和帧数，这里帧数记录为积分的时间间隔 */
    delta_t_ = imu_array_.back().timestamp() - imu_array_.front().timestamp();
    frames_ = imu_array_.size() - 1;
}

void Preintegration::reset()
{
    if (!imu_array_.empty())
    {
        Imu last_element = std::move(imu_array_.back()); // 移动语义避免拷贝
        imu_array_.clear();
        imu_array_.push_back(std::move(last_element));
    }

    delta_t_ = 0.0;
    frames_ = 0;
    delta_pos_.setZero();
    delta_vel_.setZero();
    delta_rot_.setIdentity();
    cov_.setZero();

    return ;
}