#ifndef _GINS_GNSS_EPH_H_
#define _GINS_GNSS_EPH_H_

#include <map>

#include "rtklib/rtklib.h"
#include "eigen_defs.h"
#include "interface.h"
#include "f2b0_type.h"
#include "gins.h"

constexpr double EARTH_MU = 3.986004418e14;      // BDS地球引力常数(m^3/s^2)
constexpr double EARTH_OMEGA_E = 7.2921150e-5;    // BDS地球自转角速度(rad/s)

constexpr double C_LIGHT = 299792458.0; // 光速(m/s)

class GnssEphBDS
{
public:
    GnssEphBDS() = default;
    ~GnssEphBDS() = default;
    void addPeph(const F2B0::PephBDS &peph) { peph_map_[peph.svid] = peph; }
    void addPalm(const F2B0::PalmBDS &palm) { palm_map_[palm.svid] = palm; }

    /**
     * @brief 更新卫星信息，没必要计算所有卫星，故增加了卫星ID作为参数
     */
    void updateSatInfo(const uint8_t svid, const double t);
    /** 注意这里的map不能用[]来访问，因为[]属于非const操作，当key不存在时会创建的 */
    F2B0::PephBDS peph(const uint8_t svid) const { return peph_map_.at(svid); };
    Vec3d ecefPos(const uint8_t svid) const { return sat_info_map_.at(svid).ecefPos; };
    Vec3d ecefVel(const uint8_t svid) const { return sat_info_map_.at(svid).ecefVel; };
    double pAcc(const uint8_t svid) const { return sat_info_map_.at(svid).pAcc; };
    double vAcc(const uint8_t svid) const { return sat_info_map_.at(svid).vAcc; };
    double errClk(const uint8_t svid) const { return sat_info_map_.at(svid).errClk; };
    double clkDrift(const uint8_t svid) const { return sat_info_map_.at(svid).clkDrift; };

private:
    /** map 可以天然避免重复插入 */
    std::map<uint8_t, F2B0::PephBDS> peph_map_;  /** 星历数据 */
    std::map<uint8_t, F2B0::PalmBDS> palm_map_;  /** 历书数据 */
    std::map<uint8_t, PephSatInfo> sat_info_map_;  /** 卫星信息 */
};

#endif /** _GINS_GNSS_H_ */
