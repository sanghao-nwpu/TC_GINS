
#include "eigen_defs.h"
#include "gnss_solver.h"

using namespace F2B0;


bool GnssSolver::spp()
{
    const int min_sats = 5; // 至少需要5颗卫星
    const int n = sat_info_map_.size();
    if (n < min_sats)
    {
        return false;
    }

    uint16_t svid = 0;
    uint16_t sigid = 0; /** signal_id */
    Eigen::MatrixXd H(2 * n, 8);    // 设计矩阵：8个状态量
    Eigen::VectorXd b(2 * n);       // 观测残差

    // 初始猜测（可设为地球中心或上次解算结果）
    Vec3d rec_pos = Vec3d::Zero();
    Vec3d rec_vel = Vec3d::Zero();
    double clk_err = 0.0;
    double clk_drift = 0.0;
    if (has_approx_llh_)
    {
        rec_pos = approx_llh_;
    }

    // 迭代解算（最多10次）
    /** TODO: 这里应该可以解析求解的，没必要迭代 */
    for (int iter = 0; iter < 10; ++iter) {
        // 遍历观测，构建设计矩阵和残差
        for (auto it = sat_info_map_.begin(); it != sat_info_map_.end(); ++iter)
        {
            int i = 0;
            svid = it->first;
            sigid = it->second.sigid;/** TODO: 这里的signal_id暂时的，手册未找到signal_id字段 */

            // ---- 伪距观测方程 ----
            double geometric_dist = (rec_pos - it->second.satPos).norm();
            H.block<1, 3>(i, 0) = it->second.e;
            H(i, 6) = 1; // clk_err
            b(i) = it->second.pseudorange - (geometric_dist + it->second.errClk);

            // ---- 多普勒观测方程 ----
            double relative_vel = (it->second.satVel - rec_vel).dot(it->second.e);
            H.block<1, 3>(n + i, 0) = -it->second.e;
            H(n + i, 7) = 1.0;      // clk_drift
            b(n + i) = it->second.pseudorangeRate - (-it->second.freq * relative_vel + it->second.clkDrift);
            
            i++;
        }

        // 最小二乘求解：delta_x = (H^T H)^-1 H^T b
        Eigen::VectorXd dx = (H.transpose() * H).ldlt().solve(H.transpose() * b);

        // 更新状态
        rec_pos += dx.head<3>();
        rec_vel += dx.segment<3>(3);
        clk_err += dx(6);
        clk_drift += dx(7);

        // 收敛判断（位置变化<0.1m，速度变化<0.01m/s）
        if (dx.head<6>().norm() < 0.1) {
            return true;
        }
    }
    return false;
}

void GnssSolver::updateSatInfoMap()
{
    uint8_t svid = 0;
    uint32_t obs_num = nav_sv_info_.numCh;
    
    /** 清空观测地图 */
    sat_info_map_.clear();
    for (int i = 0; i < nav_sv_info_.numCh; ++i) 
    {
        svid = nav_sv_info_.svInfo[i].svid;
        
        /** 若不存在卫星星历数据，则跳过 */
        if (peph_map_.find(svid) == peph_map_.end())
        {
            continue;
        }

        /** 赋值伪距信息 */        
        sat_info_map_[svid].pseudorange = nav_sv_info_.svInfo[i].pseudorange;
        sat_info_map_[svid].pseudorangeRate = nav_sv_info_.svInfo[i].pseudorangeRate;
        sat_info_map_[svid].prRes = nav_sv_info_.svInfo[i].prRes;

        /** 解析观测的信号与频率 TODO: 可能是sysmask的函数 */
        sat_info_map_[svid].sigid = nav_clock2_.satClk[i].sysmask;
        sat_info_map_[svid].freq = 10 * sat_info_map_[svid].sigid;

        /** 根据星历计算卫星的位置和速度 */
        Vec3d satPos = Vec3d::Zero();
        Vec3d satVel = Vec3d::Zero();
        calculateSatPosVel(svid, satPos, satVel);
        sat_info_map_[svid].iTow = nav_sv_info_.iTow;
        // sat_info_map_[svid].gnssid = nav_sv_info_.svInfo[i].gnssid;
        sat_info_map_[svid].svid = svid;
        sat_info_map_[svid].satPos = satPos;
        sat_info_map_[svid].satVel = satVel;

        /** 计算卫星的时钟偏差和漂移 */
        double clk_err = 0.0;
        double clk_drift = 0.0;
        calculateSatClkErrDrift(svid, clk_err, clk_drift);
        sat_info_map_[svid].errClk = clk_err;
        sat_info_map_[svid].clkDrift = clk_drift;

        /** 赋值卫星方位角和高度角，计算卫星的方向矢量 */
        double azim = nav_sv_info_.svInfo[i].azim * M_PI / 180.0;
        double elev = nav_sv_info_.svInfo[i].elev * M_PI / 180.0;
        sat_info_map_[svid].azim = azim;
        sat_info_map_[svid].elev = elev;
        sat_info_map_[svid].e.x() = cos(azim) * cos(elev);
        sat_info_map_[svid].e.y() = sin(azim) * cos(elev);
        sat_info_map_[svid].e.z() = sin(elev);

        /** 计算卫星观测的电离层延迟 */
        sat_info_map_[svid].ionoDelay = calculateIonoDelay(svid);
    }
    return;
}

void GnssSolver::calculateSatPosVel(const uint8_t svid, Vec3d &pos, Vec3d &vel)
{
    // 1. 解码星历参数（转换为标准单位）
    const double sqrtA = peph_map_.at(svid).sqrtA * pow(2, -19);
    const double e = peph_map_.at(svid).e * pow(2, -33);
    const double M0 = peph_map_.at(svid).M0 * (pow(2, -31) * M_PI);
    const double DeltaN = peph_map_.at(svid).DeltaN * (pow(2, -43) * M_PI);
    const double toe = peph_map_.at(svid).toe * pow(2, 4);
    const double i0 = peph_map_.at(svid).i0 * (pow(2, -31) * M_PI);
    const double iDot = peph_map_.at(svid).iDot * (pow(2, -43) * M_PI);
    const double Omega0 = peph_map_.at(svid).Omega0 * (pow(2, -31) * M_PI);
    const double OmegaDot = peph_map_.at(svid).OmegaDot * (pow(2, -43) * M_PI);
    const double w = peph_map_.at(svid).w * (pow(2, -31) * M_PI);
    const double Cuc = peph_map_.at(svid).Cuc * pow(2, -29);
    const double Cus = peph_map_.at(svid).Cus * pow(2, -29);
    const double Crc = peph_map_.at(svid).Crc * pow(2, -5);
    const double Crs = peph_map_.at(svid).Crs * pow(2, -5);
    const double Cic = peph_map_.at(svid).Cic * pow(2, -29);
    const double Cis = peph_map_.at(svid).Cis * pow(2, -29);
    const double toc = peph_map_.at(svid).toc * pow(2, 4);
    const double af0 = peph_map_.at(svid).af0 * pow(2, -31);
    const double af1 = peph_map_.at(svid).af1 * pow(2, -43);
    const double af2 = peph_map_.at(svid).af2 * pow(2, -55);
    const double tgd = peph_map_.at(svid).tGD * pow(2, -31);  
    const double t = nav_sv_info_.iTow * pow(2, -32);

    // 2. 计算归化时间tk
    double tk = t - toe;
    if (tk > 302400) tk -= 604800;
    else if (tk < -302400) tk += 604800;

    // 3. 计算平近点角M
    const double A = sqrtA * sqrtA;
    const double n0 = sqrt(EARTH_MU / (A * A * A));
    const double n = n0 + DeltaN;
    const double M = M0 + n * tk;

    // 4. 解开普勒方程求偏近点角E
    double E = M;
    for (int i = 0; i < 10; ++i) {
        double E_old = E;
        E = M + e * sin(E);
        if (fabs(E - E_old) < 1e-12) break;
    }

    // 5. 计算真近点角f
    const double sinE = sin(E);
    const double cosE = cos(E);
    const double nu = atan2(sqrt(1 - e * e) * sinE, cosE - e);

    // 6. 计算升交点角距Phi
    const double Phi = nu + w;

    // 7. 计算二阶修正项
    const double du = Cus * sin(2 * Phi) + Cuc * cos(2 * Phi);
    const double dr = Crs * sin(2 * Phi) + Crc * cos(2 * Phi);
    const double di = Cis * sin(2 * Phi) + Cic * cos(2 * Phi);

    // 8. 计算修正后的轨道参数
    const double u = Phi + du;
    const double r = A * (1 - e * cosE) + dr;
    const double i = i0 + di + iDot * tk;

    // 9. 计算卫星在轨道面坐标
    const double x_prime = r * cos(u);
    const double y_prime = r * sin(u);

    // 10. 计算升交点经度Omega (MEO/IGSO卫星 考虑地球自转)
    const double Omega = Omega0 + (OmegaDot - EARTH_OMEGA_E) * tk - EARTH_OMEGA_E * toe;

    // 11. 计算ECEF坐标 (MEO/IGSO卫星)
    pos.x() = x_prime * cos(Omega) - y_prime * cos(i) * sin(Omega);
    pos.y() = x_prime * sin(Omega) + y_prime * cos(i) * cos(Omega);
    pos.z() = y_prime * sin(i);

    /** ----------------------------------------------- */
    /** ---------------------计算速度--------------------- */
    /** ----------------------------------- */
    // 2. 计算偏近点角变化率
    const double E_dot = n / (1 - e * cosE);

    // 3. 计算真近点角变化率
    const double nu_dot = E_dot * sqrt(1 - e * e) / (1 - e * cosE);

    // 4. 计算升交点角距变化率
    const double u_dot = nu_dot + 2 * (Cus * cos(2 * Phi) - Cuc * sin(2 * Phi)) * nu_dot;

    // 5. 计算径向和切向速度
    const double r_dot = A * e * sinE * E_dot + 2 * (Crs * cos(2 * Phi) - Crc * sin(2 * Phi)) * nu_dot;
    const double x_prime_dot = r_dot * cos(u) - r * u_dot * sin(u);
    const double y_prime_dot = r_dot * sin(u) + r * u_dot * cos(u);

    // 6. 计算倾角变化率
    const double i_dot = iDot + 2 * (Cis * cos(2 * Phi) - Cic * sin(2 * Phi)) * nu_dot;

    // 7. 计算升交点经度变化率（考虑地球自转）
    const double Omega_dot = OmegaDot - EARTH_OMEGA_E;

    // 8. 计算ECEF系速度
    /** TODO: 这里的vx和vy与csdn不一致，可能还是要推导一下 */
    vel.x() = x_prime_dot * cos(Omega) - y_prime_dot * cos(i) * sin(Omega) + y_prime * Omega_dot * sin(Omega) + y_prime * sin(i) * sin(Omega) * i_dot;
    vel.y() = x_prime_dot * sin(Omega) + y_prime_dot * cos(i) * cos(Omega) - y_prime * Omega_dot * cos(Omega) - y_prime * sin(i) * cos(Omega) * i_dot;
    vel.z() = y_prime_dot * sin(i) + y_prime * cos(i) * i_dot;

    return ;
}

/** @brief 计算卫星时钟 */
void GnssSolver::calculateSatClkErrDrift(const uint8_t svid, double &errClk, double &clkDrift)
{
    const double toc = peph_map_.at(svid).toc * pow(2, 4);
    const double af0 = peph_map_.at(svid).af0 * pow(2, -31);
    const double af1 = peph_map_.at(svid).af1 * pow(2, -43);
    const double af2 = peph_map_.at(svid).af2 * pow(2, -55);
    const double tgd = peph_map_.at(svid).tGD * pow(2, -31);  
    const double t = nav_sv_info_.iTow * pow(2, -32);

    // ---- 钟差和钟漂计算 ----
    double dt = t - toc;
    if (dt > 302400) dt -= 604800;
    else if (dt < -302400) dt += 604800;
    /** TODO: csdn上还有一个dtr项，这里没有考虑 */
    errClk = af0 + af1 * dt + af2 * dt * dt - tgd;
    clkDrift = af1 + 2 * af2 * dt;

    return ;
}

double GnssSolver::calculateIonoDelay(const uint16_t svid)
{
    const double elev = sat_info_map_.at(svid).elev;
    const double alpha0 = peph_map_.at(svid).Alpha0;
    const double alpha1 = peph_map_.at(svid).Alpha1;
    const double alpha2 = peph_map_.at(svid).Alpha2;
    const double alpha3 = peph_map_.at(svid).Alpha3;
    const double beta0 = peph_map_.at(svid).Beta0;
    const double beta1 = peph_map_.at(svid).Beta1;
    const double beta2 = peph_map_.at(svid).Beta2;
    const double beta3 = peph_map_.at(svid).Beta3;

    Vec3d llh = Vec3d::Zero();
    if (has_approx_llh_)
    {
        llh = approx_llh_;
    }
    else if (gnss_sol_success_)
    {
        llh = gnss_sol_.llh;
    }
    // 1. 计算电离层穿刺点位置
    double psi = 0.0137 / (elev + 0.11) - 0.022;  // 地磁纬度修正
    double phi_ip = llh.x() + psi * cos(llh.y());          // 穿刺点纬度
    double lambda_ip = llh.y() + psi * sin(llh.y()) / cos(phi_ip); // 穿刺点经度

    // 2. 计算当地时间（秒）
    double t = 43200 * lambda_ip / M_PI + nav_sv_info_.iTow / 1000.0;  // 转换为穿刺点地方时
    t = fmod(t, 86400);                          // 归一化到0-86400秒

    // 3. 使用Alpha/Beta系数计算天顶延迟
    double I_zenith = alpha0 
                    + alpha1 * cos(2 * M_PI * (t - beta0) / 86400)
                    + alpha2 * cos(4 * M_PI * (t - beta1) / 86400)
                    + alpha3 * cos(6 * M_PI * (t - beta2) / 86400);

    // 4. 倾斜因子修正
    double F = 1.0 / sqrt(1.0 - pow(0.947 * cos(elev), 2));
    return I_zenith * F;
}
