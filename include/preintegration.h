#ifndef PREINTEGRATION_H
#define PREINTEGRATION_H

#include "imu.h"

class Preintegration 
{
private:
    double delta_t_ = 0.0;    /** 预积分时间长度 */
    uint32_t frames_ = 0;   /** 预积分用的帧数 */

    std::deque<Imu> imu_array_; /** 预积分用到的Imu数据，左闭右开原则，积分到imu_array_.back() */

    /** 预积分观测量 */
    Mat3d delta_rot_ = Mat3d::Identity();
    Vec3d delta_vel_ = Vec3d::Zero();
    Vec3d delta_pos_ = Vec3d::Zero();

    /** 预积分观测量的协方差 */
    Mat<double, 9, 9> cov_ = Mat<double, 9, 9>::Zero();

    /** 预积分用的零偏 */
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();

    /** 预积分相对于零偏的雅可比,零偏变化无需重新计算积分，用雅可比补偿即可 */
    Mat3d dr_dbg_ = Mat3d::Zero();
    Mat3d dv_dbg_ = Mat3d::Zero();
    Mat3d dv_dba_ = Mat3d::Zero();
    Mat3d dp_dba_ = Mat3d::Zero();

    /** 预计分类中不包含观测量相对于两端状态的雅可比 */
public:
    /** @brief 针对所有的IMU做预积分 */
    void reintergration();
    void addImu(const Imu &imu);
    void reset(); 
    

    Mat<double, 9, 9> cov() const { return cov_; }
    inline double delta_t() const { return delta_t_; }
    inline Vec3d delta_pos() const { return delta_pos_; }
    inline Vec3d delta_vel() const { return delta_vel_; }
    inline Mat3d delta_rot() const { return delta_rot_; }
private:
    void intergrate(const Imu &last_imu, const Imu &curr_imu);


};

#endif // PREINTEGRATION_H