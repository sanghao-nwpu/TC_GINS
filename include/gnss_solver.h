#ifndef _GINS_GNSS_SOLVER_H_
#define _GINS_GNSS_SOLVER_H_

#include "f2b0_type.h"
#include <map>

using namespace F2B0;

constexpr double EARTH_MU = 3.986004418e14;      // BDS地球引力常数(m^3/s^2)
constexpr double EARTH_OMEGA_E = 7.2921150e-5;    // BDS地球自转角速度(rad/s)
constexpr double C_LIGHT = 299792458.0; // 光速(m/s)

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

/** GNSS satellite information */
struct satInfo
{
    uint32_t iTow;   /** GPS周内时间 */
    uint8_t gnssid;  /** 卫星系统ID */
    uint8_t sigid;   /** 信号ID */
    uint8_t svid;    /** 卫星ID */
    double freq;     /** 信号频率 */
    double azim;    /** 方位角 */
    double elev;     /** 高度角 */
    Vec3d satPos;          /** 卫星ECEF坐标位置 */
    Vec3d satVel;          /** 卫星ECEF坐标速度 */
    double errClk;       /** 钟偏差 */
    double clkDrift;     /** 钟漂移 */
    Vec3d e;            /** 卫星方向矢量 */
    double ionoDelay;   /** 离子时延 */
    int32_t prRes;  /** 伪距残差(cm) */
    float pseudorangeRate;  /** 伪距率(m/s) */
    double pseudorange;  /** 伪距(m) */
};

/** 
 * @brief GNSS 求解器 
 * 
 */
class GnssSolver
{
private:

    uint8_t nav_msg_mask_ = 0;
    F2B0::NavTime nav_time_;    
    F2B0::NavTimeUTC nav_time_utc_;
    F2B0::NavClock nav_clock_;
    F2B0::NavClock2 nav_clock2_;
    F2B0::NavSvInfo nav_sv_info_;
    F2B0::NavSvState nav_sv_state_;
    F2B0::NavPVT nav_pvt_;

    /** map 可以天然避免重复插入 */
    std::map<uint8_t, F2B0::PephBDS> peph_map_;  /** 星历数据 */
    std::map<uint8_t, F2B0::PalmBDS> palm_map_;  /** 历书数据 */
    std::map<uint8_t, satInfo> sat_info_map_;  /** 卫星信息 */

    /** GNSS 定位结果 */
    NavGnssSol gnss_sol_;
    Vec3d approx_llh_;  // 接收机近似位置
    bool has_approx_llh_ = false; // 标记是否已初始化
    bool gnss_sol_success_ = false; // 标记是否成功求解GNSS位置
    
public:
    GnssSolver() = default;
    ~GnssSolver() = default;

    /** @brief 添加星历数据，直接添加到map中 */
    inline void addPeph(const F2B0::PephBDS &peph) { peph_map_[peph.svid] = peph; }

    /** @brief 添加历书数据，直接添加到map中 */
    inline void addPalm(const F2B0::PalmBDS &palm) { palm_map_[palm.svid] = palm; }

    /** @brief 添加卫星观测信息 */
    inline void addNavSvInfo(const F2B0::NavSvInfo &nav_sv_info) { nav_sv_info_ = nav_sv_info; }

    /** @brief 添加卫星时钟信息 */
    inline void addNavClock2(const F2B0::NavClock2 &nav_clock2) { nav_clock2_ = nav_clock2; }

    /** @brief 添加导航数据 */
    inline void addNavPVT(const F2B0::NavPVT &nav_pvt) { nav_pvt_ = nav_pvt; }

    /** @brief 设置接收机近似位置 */
    inline void setApproxLLH(const Vec3d &approx_llh) { approx_llh_ = approx_llh; has_approx_llh_ = true; }
    
    /** @brief 计算单点定位结果 */
    bool spp();

    /** @brief 更新卫星信息地图 */
    void updateSatInfoMap();

private:
    /** @brief 计算星历数据 */
    void calculateSatPosVel(const uint8_t svid, Vec3d &pos, Vec3d &vel);

    /** @brief 计算卫星时钟 */
    void calculateSatClkErrDrift(const uint8_t svid, double &errClk, double &clkDrift);

    /** @brief 计算电离层延迟 */
    double calculateIonoDelay(const uint16_t svid);
};

#endif /* _GINS_GNSS_H_ */
