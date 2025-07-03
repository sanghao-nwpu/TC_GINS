#ifndef _MOTION_DETECTOR_H_
#define _MOTION_DETECTOR_H_

#include "imu.h"
#include "gnss_nmea.h"

constexpr float RECORD_DURATION = 5.0;

/** @brief motion detector */
class MotionDetector
{
    struct ImuRecord
    {
        double timestamp;
        Vec3d gyro;
        Vec3d acc;
    };

    struct GnssRecord
    {
        double timestamp;
        Vec3d pos;
        double vel_forward;
    };
    
    
public:
    void addImu(const Imu& imu);
    void addGnssNmea(const GnssNmea& gnss_nmea);

    /** @brief update motion status */
    void update();

    /** @brief check if the device is moving */
    bool isMoving() const { return motion_status_ == 2; }
    
    uint8_t getMotionStatus() const { return motion_status_; }
    Vec3d getGyroStatDev() const { return gyro_stat_dev_; }
    Vec3d getAccStatDev() const { return acc_stat_dev_; }
    Vec4d getGyroDistrib() const { return gyro_distrib_; }
    Vec4d getAccDistrib() const { return acc_distrib_; }

private:
    uint8_t motion_status_;  /** 0-unknown, 1-stationary, 2-moving */
    Vec3d gyro_stat_dev_;  /** standard deviation of gyro measurements */
    Vec3d acc_stat_dev_;  /** standard deviation of accelerometer measurements */
    Vec4d gyro_distrib_;  /** distribution of gyro measurements, x,y,z,w */
    Vec4d acc_distrib_;  /** distribution of accelerometer measurements, x,y,z,xy */
    std::deque<ImuRecord> imu_queue_;
    std::deque<GnssRecord> gnss_nmea_queue_;
};


#endif // _MOTION_DETECTOR_H_