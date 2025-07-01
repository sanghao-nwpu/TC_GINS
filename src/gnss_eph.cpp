#include "gnss_eph.h"

using namespace F2B0;


void GnssEphBDS::updateSatInfo(const uint8_t svid, const double t)
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
    sat_info_map_.at(svid).ecefPos.x() = x_prime * cos(Omega) - y_prime * cos(i) * sin(Omega);
    sat_info_map_.at(svid).ecefPos.y() = x_prime * sin(Omega) + y_prime * cos(i) * cos(Omega);
    sat_info_map_.at(svid).ecefPos.z() = y_prime * sin(i);

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
    sat_info_map_.at(svid).ecefVel.x() = x_prime_dot * cos(Omega) - y_prime_dot * cos(i) * sin(Omega) + y_prime * Omega_dot * sin(Omega) + y_prime * sin(i) * sin(Omega) * i_dot;
    sat_info_map_.at(svid).ecefVel.y() = x_prime_dot * sin(Omega) + y_prime_dot * cos(i) * cos(Omega) - y_prime * Omega_dot * cos(Omega) - y_prime * sin(i) * cos(Omega) * i_dot;
    sat_info_map_.at(svid).ecefVel.z() = y_prime_dot * sin(i) + y_prime * cos(i) * i_dot;

    // ---- 钟差和钟漂计算 ----
    double dt = t - toc;
    if (dt > 302400) dt -= 604800;
    else if (dt < -302400) dt += 604800;
    /** TODO: csdn上还有一个dtr项，这里没有考虑 */
    sat_info_map_.at(svid).errClk = af0 + af1 * dt + af2 * dt * dt - tgd;
    sat_info_map_.at(svid).clkDrift = af1 + 2 * af2 * dt;
}

