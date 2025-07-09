
#ifndef _GINS_INTERFACE_H_
#define _GINS_INTERFACE_H_

#include "f2b0_type.h"

struct ImuData
{
    double timestamp;
    double acc[3];
    double gyr[3];
};

struct GnssData
{
    double timestamp;
    double blh[3]; /** latitude(deg), longitude(deg), height(m) */
    double dev[3];
    double vel_forward; /** m/s */
    double vel_track;   /** deg */
    double vel_upward;  /** m/s */
    int sat_num;
    int sol_type;
};

struct Config
{
    uint8_t imu_freq;
    uint8_t gnss_nmea_freq;
    uint8_t gnss_obs_freq;
    uint8_t gins_type;      /** 0(default)-LC_GINS, 1-TC_GINS, 2-SPP, other-ERROR */
    uint8_t out_type;       /** 0(default)-llh(纬经高输出), 1-ENU */
    Mat3d imu_rot;          /** IMU相对车体的旋转外参 */
    Vec3d imu_arm;          /** IMU相对车体的平移杆臂 */
    Vec3d gnss_arm;         /** GNSS相对车体的平移杆臂 */
    Vec3d gnss_base;        /** Gnss基站原点，仅当out_type为1时，该值有效 */
    std::string imu_model;  /** IMU 型号，算法内部根据型号来配置参数 */
    std::string gnss_model; /** GNSS 型号，算法内部根据型号来配置参数 */
};

/** 导航结果 */
struct NavResult
{
    uint32_t timestamp; /** 时间戳 ms */
    double pos[3];      /** 和配置参数有关，默认输出为llh */
    float vel[3];       /** 车体系前左上 */
    float att[3];       /** 车体系右手系：roll, pitch, yaw */
    float acc[3];       /** 车体系前左上 */
    float gyro[3];      /** 车体系前左上 */
    float latAcc;       /** 横向位置精度 */
    float lonAcc;       /** 纵向位置精度 */
    float altAcc;       /** 高度精度 */
    float velAcc;       /** 速度精度 */
    float attAcc;       /** 姿态精度 */

    uint8_t navStatus;    /** 导航状态: 0-unknwon, 1-fix */
    uint8_t motionStatus; /** 运动状态: 0-unknwon, 1-静止, 2-运动 */
};

#endif /* _GINS_INTERFACE_H_ */