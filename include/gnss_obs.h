#ifndef _GINS_GNSS_OBS_H_
#define _GINS_GNSS_OBS_H_

#include "rtklib/rtklib.h"
#include "eigen_defs.h"
#include "interface.h"
#include "f2b0_type.h"

/**
 * @brief GNSS 观测数据
 * 
 */
class GnssObs
{
public:
    GnssObs() = default;
    ~GnssObs() = default;
    GnssObs(const GnssObsF2B0 &obs) : obs_(obs) {}
    const GnssObsF2B0 &obs() const { return obs_; }
    double itow() const { return obs_.nav_sv_info.iTow; }   /** 返回时间戳 */
    void set_obs(const GnssObsF2B0 &obs) { obs_ = obs; }
    uint16_t numObs() const { return obs_.nav_sv_info.numCh; }   /** 返回卫星数目 */

    /** 返回第 index 个卫星信号的观测数据 */
    F2B0::SvInfo svInfo(uint16_t index) const { return obs_.nav_sv_info.svInfo[index]; }   /** 返回第 index 个卫星的观测数据 */

    /** 返回第 index 个卫星的时钟数据 */
    F2B0::SatClk satClk(uint16_t index) const { return obs_.nav_clock2.satClk[index]; }   /** 返回第 index 个卫星的时钟数据 */

private:
    GnssObsF2B0 obs_;   /** F2B0 观测数据 */
};

#endif /* _GINS_GNSS_H_ */
