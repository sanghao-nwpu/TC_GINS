#ifndef _GINS_IMU_H_
#define _GINS_IMU_H_

#include <Eigen/Dense>

class IMU {
public:
    struct Data {
        double timestamp;  // 秒为单位
        Eigen::Vector3d accel;  // m/s²
        Eigen::Vector3d gyro;   // rad/s
    };

    // 更新数据
    void update(const Data& new_data);
    
    // 获取最新数据
    Data get_latest() const;

    // 标定零偏
    void set_bias(const Eigen::Vector3d& accel_bias, 
                 const Eigen::Vector3d& gyro_bias);

private:
    Data current_data_;
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
};

#endif  // _GINS_IMU_H_