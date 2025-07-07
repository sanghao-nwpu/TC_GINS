#include "tc_gins.h"

void TcGins::predictByPreInt()
{
    double dt = preintegration_.delta_t();
    Mat3d Ri = Mat3d::Identity();
    Mat3d Rj = Mat3d::Identity();
    Vec3d vi = Vec3d::Zero();
    Vec3d vj = Vec3d::Zero();
    Vec3d pi = Vec3d::Zero();
    Vec3d pj = Vec3d::Zero();
    Vec3d dp = preintegration_.delta_pos();
    Vec3d dv = preintegration_.delta_vel();
    Mat3d dr = preintegration_.delta_rot();

    /** 状态更新 */
    Ri = Quatd(nav_info_.q);
    vi = nav_info_.v;
    pi = nav_info_.p;

    Rj = Ri * dr;
    vj = vi + g_ * dt + dv;
    pj = pi + vi * dt + 0.5 * g_ * dt * dt + Ri * dp;

    /** 积分状态协方差更新 */
    P_.block<9, 9>(0, 0) += preintegration_.cov();

    nav_info_.q = Quatd(Rj);
    nav_info_.v = vj;
    nav_info_.p = pj;
}

void TcGins::gnssObsUpdate()
{
    uint16_t num_obs = gnss_solver_.numObs();
    MatXd H(2 * num_obs, TC_GINS_DIM);
    VecXd Z(2 * num_obs);
    MatXd R(2 * num_obs, 2 * num_obs);
    Vec3d e = Vec3d::Zero();    /** 方向矢量 */

    /** 观测方程 */
    H.setZero();
    Z.setZero();
    R.setZero();

    for (uint16_t i = 0; i < num_obs; i++)
    {
        /** 伪距观测 */
        F2B0::SvInfo svinfo = gnss_solver_.svinfo(i);
        e = gnss_solver_.e(svinfo.svid, nav_info_.p);
        H.block<3, 1>(i, 0) = e;
        H(i, 15) = 1;
        Z(i) = svinfo.pseudorange;
        R(i, i) = svinfo.prRes * svinfo.prRes;

        /** 伪距率观测 */
        H.block<3, 1>(i + num_obs, 1) = e;
        H(i + num_obs, 16) = 1;
        Z(i + num_obs) = svinfo.pseudorangeRate;
        R(i + num_obs, i + num_obs) = 0.5 * 0.5;    /** TODO: 似乎数据里都没有微距率的精度信息 */
    }
    
    return ; 
}

void TcGins::processImu(const Imu &imu)
{
    /** 预积分结果 */
    Vec3d delta_pos = Vec3d::Zero();
    Vec3d delta_vel = Vec3d::Zero();
    Mat3d delta_rot = Mat3d::Identity();
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
        last_imu_ = imu_trans;  /** Eigen::Vector3d 可以直接浅拷贝，无指针变量 */
        return ;
    }

    /** motion detector 更新运动状态，内部自行控制更新频率，这里只管调用 */
    motion_detector_.update();

    /** 未初始化，直接返回 */
    if (!isInitialized())
    {
        return;
    }    

    /** 预积分处理, 每间隔一帧做一次IMU的预积分处理 */
    preintegration_.reintergration();

    /** IMU 状态递推 */
    predictByPreInt();

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

void TcGins::processGnssObs(const GnssObs& gnss_obs)
{
    /** 原始观测添加到求解器 */
    gnss_solver_.addGnssObs(gnss_obs);

    /** 求解器解算单点定位结果 */
    gnss_solver_.spp();

    if (!isInitialized())
    {
        if (gnss_solver_.isValidSPP())
        {
            initilize();
        }
        return ;
    }

    /** 已经初始化, gnss观测数据更新 */
    gnssObsUpdate();

    return ;
}

void TcGins::processGnssEph(const GnssEphBDS &gnss_eph)
{
    gnss_solver_.addGnssEph(gnss_eph);
}


void TcGins::SetConfig(const Config config)
{
    config_ = config;
    if (config_.imu_model.find("LSM6DSR") != std::string::npos)
    {
        sensor_spec_.accel_dev = {0.1, 0.1, 0.1};
        sensor_spec_.gyro_dev = {0.01, 0.01, 0.01};
    }
    else if (config_.imu_model.find("MPU6050") != std::string::npos)
    {
        sensor_spec_.accel_dev = {0.01, 0.01, 0.01};
        sensor_spec_.gyro_dev = {0.001, 0.001, 0.001};
    }
    else
    {
        std::cout << "Unknown IMU model: " << config_.imu_model << std::endl;
        std::cout << "Using default IMU parameters." << std::endl;
        sensor_spec_.accel_dev = {0.01, 0.01, 0.01};
        sensor_spec_.gyro_dev = {0.001, 0.001, 0.001};
    }
}