#ifndef _GINS_GNSS_SOLVER_H_
#define _GINS_GNSS_SOLVER_H_

#include "f2b0_type.h"
#include "gnss_obs.h"
#include "gnss_eph.h"

using namespace F2B0;

/** GNSS solution */
struct NavGnssSol
{
    int32_t tow;    /** GPS周内时间 */
    uint16_t wn;    /** GPS周数 */
    uint8_t fixflags;
    uint8_t velflags;
    uint8_t fixGnssMask;    /** 参与定位的卫星系统掩码 */
    uint8_t numSat;     /** 参与定位的卫星数 */
    uint8_t numFixBds;
    Vec3d llh;
    float pAcc;
    Vec3d vel;  /** ned坐标系下的速度 */
    float vAcc;
};


/** 
 * @brief GNSS 求解器 
 * 
 */
class GnssSolver
{
public:
    GnssSolver() = default;
    ~GnssSolver() = default;
    
    void addGnssObs(const GnssObs& obs) { obs_ = obs; };
    void addGnssEph(const GnssEphBDS& eph)
    {
        eph_bds_ = eph;
        eph_valid_ = true;
    };

    /** 
     * @brief 读取卫星信号的频率
     */
    static double freq(const uint32_t sigid) { return sigid >> 16; };

    /**
     * @brief 计算单点定位结果
     */
    bool spp();

    /**
     * @brief 计算电离层延迟
     */
    double ionoDelay(const uint16_t svid);

    /** 位置在对应卫星下的方向矢量 */
    Vec3d e(const uint8_t svid, const Vec3d& ecef);


    Vec3d getSppPos() const { return gnss_sol_.llh; };
    Vec3d getSppVel() const { return gnss_sol_.vel; };

    uint16_t numObs() const { return obs_.numObs(); };
    bool isValidSPP() const { return gnss_sol_success_; };

    SvInfo svinfo(const uint16_t svidx)
    {
        return obs_.svInfo(svidx);
    }

private:
    GnssEphBDS eph_bds_;
    GnssObs obs_;
    NavGnssSol gnss_sol_;
    Vec3d approx_llh_;  // 接收机近似位置
    bool eph_valid_ = false; // 标记是否星历数据有效
    bool has_approx_llh_ = false; // 标记是否已初始化
    bool gnss_sol_success_ = false; // 标记是否成功求解GNSS位置
};

#endif /* _GINS_GNSS_H_ */
