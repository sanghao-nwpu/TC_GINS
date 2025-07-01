#ifndef _GINS_IMU_H_
#define _GINS_IMU_H_

#include <deque>

#include "interface.h"
#include "eigen_defs.h" 

class Imu {
public:
    Imu(ImuData &data) 
    {
        timestamp_ = data.timestamp;
        accel_ = Vec3d(data.acc[0], data.acc[1], data.acc[2]);
        gyro_ = Vec3d(data.gyr[0], data.gyr[1], data.gyr[2]);
    }
    
    Imu(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro)
        : timestamp_(timestamp), accel_(accel), gyro_(gyro) {}
    
    double timestamp() const { return timestamp_; };

    const Vec3d &accel(const Mat3d &rot = Mat3d::Identity(),
                       const Vec3d &bias = Vec3d::Zero()) const
    {
        return rot * (accel_ - bias);
    }

    const Vec3d &gyro(const Mat3d &rot = Mat3d::Identity(),
                      const Vec3d &bias = Vec3d::Zero()) const
    {
        return rot * (gyro_ - bias);
    }

private:
    double timestamp_;
    Vec3d accel_;
    Vec3d gyro_;
};

#endif  // _GINS_IMU_H_