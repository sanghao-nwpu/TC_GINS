#include "gnss_solver.h"

using namespace F2B0;


bool GnssSolver::spp()
{
    const int min_sats = 5; // 至少需要5颗卫星
    if (obs_.numObs() < min_sats)
    {
        return false;
    }

    const int n = obs_.numObs();
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
        for (int i = 0; i < n; ++i) {
            /** TODO: 这里的signal_id暂时的，手册未找到signal_id字段 */
            svid = obs_.svInfo(i).svid;
            sigid = obs_.svInfo(i).quality;
            // 视线向量计算
            Vec3d satEcefPos = eph_bds_.ecefPos(svid);
            Vec3d satEcefVel = eph_bds_.ecefVel(svid);
            Vec3d los = (satEcefPos - rec_pos).normalized();
            double geometric_dist = (satEcefPos - rec_pos).norm();

            // ---- 伪距观测方程 ----
            H(i, 0) = -los.x();  // dx
            H(i, 1) = -los.y();  // dy
            H(i, 2) = -los.z();  // dz
            H(i, 6) = C_LIGHT;         // clk_err
            b(i) = obs_.svInfo(i).pseudorange - (geometric_dist + C_LIGHT * clk_err);

            // ---- 多普勒观测方程 ----
            double relative_vel = (satEcefVel - rec_vel).dot(los);
            H(n + i, 3) = los.x();  // dvx
            H(n + i, 4) = los.y();  // dvy
            H(n + i, 5) = los.z();  // dvz
            H(n + i, 7) = 1.0;      // clk_drift
            b(n + i) = obs_.svInfo(i).pseudorangeRate - (-freq(sigid) * relative_vel / C_LIGHT + clk_drift);
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
double GnssSolver::ionoDelay(const uint16_t svid)
{
    const double elev = obs_.svInfo(svid).elev;
    const double alpha0 = eph_bds_.peph(svid).Alpha0;
    const double alpha1 = eph_bds_.peph(svid).Alpha1;
    const double alpha2 = eph_bds_.peph(svid).Alpha2;
    const double alpha3 = eph_bds_.peph(svid).Alpha3;
    const double beta0 = eph_bds_.peph(svid).Beta0;
    const double beta1 = eph_bds_.peph(svid).Beta1;
    const double beta2 = eph_bds_.peph(svid).Beta2;
    const double beta3 = eph_bds_.peph(svid).Beta3;

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
    double t = 43200 * lambda_ip / M_PI + obs_.itow();  // 转换为穿刺点地方时
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
