#include "tc_gins.h"

void TcGins::updateImu(const Imu &imu)
{
    /** 构造转换外参后的IMU数据 */
    Imu imu_trans(imu.timestamp(), imu.accel(), imu.gyro(), config_.imu_rot);
    
    /** 添加数据 */
    /** imu数据收集到监测器中 */
    motion_detector_.addImu(imu_trans);
    /** 添加到预积分器中 */
    preintegration_.addImu(imu_trans);
    
    /** 仅有单帧IMU数据，无需处理，跳过 */
    if (!last_imu_flag_)
    {
        last_imu_flag_ = true;
        last_imu_ = imu_trans;
        return ;
    }


    /** motion detector 处理, 每间隔一帧做一次IMU的运动检测 */
    motion_detector_.update();

    /** 未初始化，直接返回 */
    if (!isInitialized())
    {
        return;
    }    

    /** 预积分处理, 每间隔一帧做一次IMU的预积分处理 */
    preintegration_.reintergration();

    /** IMU 状态及协方差预测,用上预积分的数据 */
    predict();

    /** 虚拟测量更新 */
    if (motion_detector_.isMoving())
    {
        nhcUpdate();
    }
    else
    {
        zuptZihrUpdate();
    }

    /** 处理完毕，移动IMU数据 */
    last_imu_ = imu_trans;
}

void TcGins::updateGnssObs(const GnssObs& gnss_obs)
{
    gnss_solver_.addGnssObs(gnss_obs);

}

void TcGins::updateGnssEph(const GnssEphBDS &gnss_eph)
{
    gnss_solver_.addGnssEph(gnss_eph);
}