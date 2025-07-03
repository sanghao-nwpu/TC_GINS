#ifndef PREINTEGRATION_H
#define PREINTEGRATION_H

#include "imu.h"

class Preintegration 
{
public:
    /** @brief 针对所有的IMU做预积分 */
    void reintergration();
    void addImu(const Imu &imu);
private:
    void intergrate(const Imu &last_imu, const Imu &curr_imu);

private:
    double delta_t_ = 0.0;    /** 预积分时间长度 */
    uint32_t frames_ = 0;   /** 预积分用的帧数 */

    /** 预积分观测量 */
    Mat3d delta_rot_ = Mat3d::Identity();
    Vec3d delta_vel_ = Vec3d::Zero();
    Vec3d delta_pos_ = Vec3d::Zero();

    /** 预积分观测量的协方差 */
    Eigen::Matrix<double, 9, 9> cov_ = Eigen::Matrix<double, 9, 9>::Zero();

    /** 预计分用的零偏 */
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();

    /** 预积分相对于零偏的雅可比,零偏变化无需重新计算积分，用雅可比补偿即可 */
    Mat3d dr_dbg_ = Mat3d::Zero();
    Mat3d dv_dbg_ = Mat3d::Zero();
    Mat3d dv_dba_ = Mat3d::Zero();
    Mat3d dp_dba_ = Mat3d::Zero();

};

#endif // PREINTEGRATION_H