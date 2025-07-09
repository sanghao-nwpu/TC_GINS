
#include <vector>

#include "motion_detector.h"

void MotionDetector::addImu(const Imu &imu)
{
    imu_queue_.push_back(imu);

    if (imu_queue_.size() > RECORD_SIZE)
    {
        imu_queue_.pop_front();
    }
}

void MotionDetector::updateMotionInfo()
{
    if (imu_queue_.size() < 100)
    {
        return ;
    }

    const int n = imu_queue_.size();
    const int left_id = floor(n * 0.025);
    const int right_id = floor(n * 0.975);
    Vec3d acc_mean = Vec3d::Zero();
    Vec3d gyro_mean = Vec3d::Zero();
    Vec3d acc_std = Vec3d::Zero();
    Vec3d gyro_std = Vec3d::Zero();
    Vec4d acc_distrib = Vec4d::Zero();
    Vec4d gyro_distrib = Vec4d::Zero();
    std::vector<double> axs, ays, azs;
    std::vector<double> gxs, gys, gzs;

    for (auto imu : imu_queue_)
    {
        axs.push_back(imu.accel().x());
        ays.push_back(imu.accel().y());
        azs.push_back(imu.accel().z());
        gxs.push_back(imu.gyro().x());        
        gys.push_back(imu.gyro().y());        
        gzs.push_back(imu.gyro().z());
    }

    /** 计算均值 */
    for (int i = 0; i < n; i++)
    {
        acc_mean += imu_queue_[i].accel();
        gyro_mean += imu_queue_[i].gyro();
    }
    acc_mean /= n;
    gyro_mean /= n;

    /** 计算标准差 */
    for (int i = 0; i < n; i++)
    {
        acc_std += (imu_queue_[i].accel() - acc_mean).cwiseAbs2();
        gyro_std += (imu_queue_[i].gyro() - gyro_mean).cwiseAbs2();
    }
    acc_std = (acc_std / n).cwiseSqrt();
    gyro_std = (gyro_std / n).cwiseSqrt();

    /** 计算分布 */
    std::sort(gxs.begin(), gxs.end());
    std::sort(gys.begin(), gys.end());
    std::sort(gzs.begin(), gzs.end());
    std::sort(axs.begin(), axs.end());
    std::sort(ays.begin(), ays.end());
    std::sort(azs.begin(), azs.end());

    acc_distrib(0) = axs[right_id]- axs[left_id];
    acc_distrib(1) = ays[right_id]- ays[left_id];
    acc_distrib(2) = azs[right_id]- azs[left_id];
    gyro_distrib(0) = gxs[right_id]- gxs[left_id];
    gyro_distrib(1) = gys[right_id]- gys[left_id];
    gyro_distrib(2) = gzs[right_id]- gzs[left_id];

    /** 赋值 */
    gyro_mean_ = gyro_mean;
    acc_mean_ = acc_mean;
    gyro_dev_ = gyro_std;
    acc_dev_ = acc_std;
    gyro_distrib_ = gyro_distrib;
    acc_distrib_ = acc_distrib;

    if (!thre_valid_)
    {
        return ;
    }

    /** 判断是否静止 */
    motion_status_ = 1;
    if (acc_distrib(0) + 0.1 < acc_distrib_stat_thre_(0))
    {
        motion_status_ = 2;
    }

    return ;
}