#ifndef _MOTION_DETECTOR_H_
#define _MOTION_DETECTOR_H_

#include "imu.h"
#include "gnss_nmea.h"

constexpr float RECORD_DURATION = 5.0;
constexpr uint32_t RECORD_SIZE = 250;

/** 
 * @brief motion detector 
 * @details detect motion status based on IMU and GNSS data
 * @note truck maybe unwork
 * */
class MotionDetector
{
private:
    double last_update_time_;   /** last update time */
    uint8_t motion_status_; /** 0-unknown, 1-stationary, 2-moving */
    bool thre_valid_;  /** threshold initialized */
    Vec3d gyro_mean_;  /** mean of gyro measurements */
    Vec3d acc_mean_;   /** mean of accelerometer measurements */
    Vec3d gyro_dev_;   /** standard deviation of gyro measurements */
    Vec3d acc_dev_;    /** standard deviation of accelerometer measurements */
    Vec4d gyro_distrib_;    /** distribution of gyro measurements, x,y,z,w */
    Vec4d acc_distrib_;     /** distribution of accelerometer measurements, x,y,z,xy */

    Vec3d gyro_stat_mean_;  /** mean of gyro measurements in stationary period */
    Vec3d acc_stat_mean_;   /** mean of accelerometer measurements in stationary period */
    Vec4d gyro_distrib_stat_thre_;    /** distribution of gyro measurements in stationary period, x,y,z,w */
    Vec4d acc_distrib_stat_thre_;     /** distribution of accelerometer measurements in stationary period, x,y,z,xy */
    std::deque<Imu> imu_queue_;

public:
    void addImu(const Imu &imu);

    /** @brief update motion status */
    void updateMotionInfo();

    /** @brief check if the device is moving */
    bool isMoving() const { return motion_status_ == 2; }

    uint8_t getMotionStatus() const { return motion_status_; }
    
    void setAccDistribStatThre(const Vec4d &acc_distrib_stat_thre) { acc_distrib_stat_thre_ = acc_distrib_stat_thre; }
    void setGyroDistribStatThre(const Vec4d &gyro_distrib_stat_thre) { gyro_distrib_stat_thre_ = gyro_distrib_stat_thre; }
    
    Vec3d gyroMean() const { return gyro_mean_; }
    Vec3d accMean() const { return acc_mean_; }
    Vec3d gyroDev() const { return gyro_dev_; }
    Vec3d accDev() const { return acc_dev_; }
    Vec4d gyroDistrib() const { return gyro_distrib_; }
    Vec4d accDistrib() const { return acc_distrib_; }

};

#endif // _MOTION_DETECTOR_H_