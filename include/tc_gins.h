
#ifndef GINS_H
#define GINS_H

/**
 * @file gins.h
 * 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-30
 * 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include <stdint.h>
#include <deque>
#include <iostream>

#include "eigen_defs.h"

/** 传感器数据头文件 */
#include "gnss_solver.h"
#include "preintegration.h"
#include "motion_detector.h"

#define MAX_SAT_NUM_BDS (64)    /** 最大卫星数 */
#define MAX_SIGNAL_NUM_BDS (5)  /** 单颗卫星最大信号数 */

/** TC-GINS 状态维度：{rot, pos, vel, bg, ba, dbg, c*dt, c*dtr} */
constexpr uint8_t TC_GINS_DIM = 20;  /** TC-GINS 状态维度 */

struct MotionInfo
{
    double timestamp;
    Vec3d acc;  /** 车体系前左上 */
    Vec3d gyro; /** 车体系前左上 */
    double vel_forward; /** 区分前后车速 */
    uint8_t motion_status;  /** 运动状态: 0-unknwon, 1-静止, 2-运动 */
};

/** 传感器参数 */
struct SensorSpec
{
    /** IMU均方差主要用来判断静止，滤波器噪声算法自适应动态设置 */
    Vec3d gyro_dev;    /** 加计算法噪声 */
    Vec3d accel_dev;   /** 陀螺仪噪声 */
};

/** 导航名义状态: {quat, posi, velo, gyro_bias, accel_bias, clock_offset, clock_drift} */
struct NavInfo
{
    double t;
    Quatd q;     /** attitude */
    Vec3d p;     /** enu frame */
    Vec3d v;     /** enu frame */
    Vec3d bg;     /**  */ 
    Vec3d ba;
    Vec3d dgb;
    double cdt;    /** 光速乘以接收机钟差 */
    double ddtr;     /** 光速乘以接收机时钟漂移 */

    Vec3d p_dev;
    Vec3d v_dev;
    Vec3d q_dev;
    Vec3d bg_dev;
    Vec3d ba_dev;
    Vec3d dgb_dev;
    double cdt_dev;
    double cdtr_dev;
};
class TcGins 
{
private:
    Config config_;
    Imu last_imu_;  /** 不需要累积数据，累积数据放在MotionDetector即可 */
    SensorSpec sensor_spec_;  /** 传感器噪声 */
    Vec3d g_ = {0, 0, -9.8};  /** 重力加速度, 前左上 */
    bool last_imu_flag_ = false;
    uint8_t flags_ = 0;

    // GnssNmea gnss_nmea_;  /** GNSS NMEA数据，用于松耦合 */
    GnssSolver gnss_solver_;    /** 这里gnss solver中已经有了obs类和EPH类了 */
    Preintegration preintegration_; /** 预积分 */ 
    MotionDetector motion_detector_;    /** 运动检测 */

    MotionInfo motion_info_;  /** 运动信息 */
    NavInfo nav_info_;  /** 导航名义状态 */
    Vec<double, TC_GINS_DIM> X_;  /** TC-GINS 状态 */
    Mat<double, TC_GINS_DIM, TC_GINS_DIM> P_;  /** TC-GINS 状态协方差 */

public:
    TcGins(/* args */);
    ~TcGins();
    void SetConfig(const Config config);
    void processImu(const Imu& imu);
    void processGnssObs(const GnssObs& gnss_obs);
    void processGnssEph(const GnssEphBDS& gnss_eph);

private:

    /** 通过预积分预测 */
    void predictByPreInt(); 
    // void nhcUpdate();
    // void zuptZihrUpdate();
    void gnssObsUpdate();

    /** @brief 返回是否已经初始化的标识 */
    bool isInitialized();

    /** 初始化滤波器 */
    void initilize();


};


#endif // GINS_H