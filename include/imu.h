#ifndef _GINS_IMU_H_
#define _GINS_IMU_H_

#include <deque>

#include "interface.h"
#include "eigen_defs.h" 

class Imu {
public:
    /** 从接口定义直接构造 */
    Imu(ImuData &data) 
    {
        timestamp_ = data.timestamp;
        accel_ = Vec3d(data.acc[0], data.acc[1], data.acc[2]);
        gyro_ = Vec3d(data.gyr[0], data.gyr[1], data.gyr[2]);
    }

    /** 从数据直接构造 */
    Imu(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro)
        : timestamp_(timestamp), accel_(accel), gyro_(gyro) {}

    /** 直接构造转换后的数据 */
    Imu(double timestamp, const Vec3d &accel, const Vec3d &gyro,
        const Mat3d &rot = Mat3d::Identity(), 
        const Vec3d &gyro_bias = Vec3d::Zero(), 
        const Vec3d &accel_bias = Vec3d::Zero())
        : timestamp_(timestamp)
    {
        accel_ = rot * (accel_ - accel_bias);
        gyro_ = rot * (gyro_ - gyro_bias);
    }

    /** 获取时间戳 */
    double timestamp() const { return timestamp_; };

    /** 获取转换后的陀螺仪数据 */
    const Vec3d gyro(const Mat3d &rot = Mat3d::Identity(),
                      const Vec3d &bias = Vec3d::Zero()) const
    {
        return rot * (gyro_ - bias);
    }

    /** 获取转换后的加速度数据 */
    const Vec3d accel(const Mat3d &rot = Mat3d::Identity(),
                       const Vec3d &bias = Vec3d::Zero()) const
    {
        return rot * (accel_ - bias);
    }
    

private:
    double timestamp_;
    Vec3d accel_;
    Vec3d gyro_;
};

#endif  // _GINS_IMU_H_