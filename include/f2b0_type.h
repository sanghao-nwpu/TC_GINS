#ifndef _F2B0_TYPE_H_
#define _F2B0_TYPE_H_

#include <stdint.h>

#define F2B0_MAX_SATELLITE_NUM (64)

namespace F2B0 {

struct NavTime
{
    uint8_t nav_sys;
    uint8_t flag;
    int16_t fractow;  /** gnss 周内秒(ns) */
    uint32_t ref_tow;  /** gnss 基准时间(ms) */
    uint16_t week;  /** gnss 周数 */
    int16_t leap_sec;  /** UTC闰秒数(s) */
    uint32_t time_err;  /** 时间误差(ns) */
};


struct NavTimeUTC
{
    uint32_t iTow;  /** GNSS周内秒(ms) */
    uint32_t tAcc;  /** 时间精度(ns) */
    int32_t nano;  /** 纳秒UTC(ns) */
    uint16_t year;  /** 年(1999~2099) */
    uint8_t month;  /** 月 */
    uint8_t day;  /** 日 */
    uint8_t hour;  /** 时 */
    uint8_t minute;  /** 分 */
    uint8_t second;  /** 秒 */
    uint8_t valid;  /** 有效标志 */
};

/** 接收机时钟 */
struct NavClock
{
    uint32_t iTow;  /** GNSS周内秒(ms) */
    int32_t clkB;  /** 基准时钟偏移(ns) */
    int32_t clkD;  /** 每秒时钟偏移(ns/s) */
    uint32_t tAcc;  /** 时钟精度(ns) */
    uint32_t fAcc;  /** 频率精度(ps/s) */
};

struct SatClk
{
    uint32_t sysmask;  /** 信号偏移位阈值 */
    int32_t clkB;  /** 基准时钟偏移(ns) */
    uint32_t tAcc;  /** 时钟精度(ns) */
};


struct NavClock2
{
    uint32_t iTow;  /** GNSS周内秒(ms) */
    uint32_t numClk;  /** 卫星时钟个数 */
    SatClk satClk[F2B0_MAX_SATELLITE_NUM];  /** 卫星时钟 */
};

/** 卫星信息 */
/** TODO: 这里应该是卫星信号信息，但是没有找到信号ID相关的定义 */
struct SvInfo
{
    uint16_t svid;  /** 卫星ID */
    int8_t flags;  /** 位阈值（参考手册） */
    int8_t quality;  /** 位字段（参考手册） */
    uint8_t cno;  /** 信噪比(dbHz) */
    int8_t elev;  /** 仰角(deg) */
    int16_t azim;  /** 方位角(deg) */
    int32_t prRes;  /** 伪距残差(cm) */
    float pseudorangeRate;  /** 伪距率(m/s) */
    double pseudorange;  /** 伪距(m) */
};


struct NavSvInfo
{
    uint32_t iTow;  /** GNSS周内秒(ms) */
    uint32_t numCh; /** 通道数量 */
    SvInfo svInfo[F2B0_MAX_SATELLITE_NUM];  /** 卫星信息 */
};

struct SvState
{
    uint16_t svid;  /** NMEA格式卫星ID */
    uint8_t eph_state;  /** 4/2/2: eph可用性/sv可见性/sv运行状况 */
    uint8_t alm_state;  /** 4/2/2: alm可用性/alm源/eph源 */
};

struct NavSvState
{
    uint32_t iTow;  /** GNSS周内秒(ms) */
    uint32_t numSv;  /** 卫星数量 */
    uint8_t rev;  /** 预留 */
    SvState svState[F2B0_MAX_SATELLITE_NUM];  /** 卫星状态 */
};

struct NavPVT
{
    uint32_t iTow;  /** GNSS周内秒(ms) */
    uint16_t year;  /** 年(1999~2099) */
    uint8_t month;  /** 月 */
    uint8_t day;  /** 日 */
    uint8_t hour;  /** 时 */
    uint8_t minute;  /** 分 */
    uint8_t second;  /** 秒 */
    uint8_t valid;  /** 信息状态 */
    uint32_t tAcc;  /** 时间精度(ns) */
    int32_t nano;  /** 纳秒UTC(-10^9~10^9, ns) */
    uint8_t fixType;  /** 定位类型: 0-无效, 1-仅航位推测, 2-2D, 3-3D, 4-gnss融合航位推测, 5-仅定位时间 */
    uint8_t res1;  /** 保留 */
    uint8_t res2;  /** 保留 */
    uint8_t numSV;  /** 卫星数量 */
    int32_t lon;  /** 经度(1e-7deg) */
    int32_t lat;  /** 纬度(1e-7deg) */
    int32_t height;  /** 椭球面以上高度(mm) */
    int32_t hMSL;  /** 平均海平面以上高度(mm) */
    uint32_t hAcc;  /** 水平精度(mm) */
    uint32_t vAcc;  /** 垂直精度(mm/s) */
    int32_t velN;  /** 北向速度分量(mm/s) */
    int32_t velE;  /** 东向速度分量(mm/s) */
    int32_t velD;  /** 地向速度分量(mm/s) */
    int32_t gSpeed;  /** 对地速度(2D,mm/s) */
    int32_t headMot;  /** 运动方向(1e-3) */
    uint32_t sAcc;  /** 速度精度() */
    uint32_t headAcc;  /** 方向精度(1e-5) */
    uint16_t pDOP;  /** 位置精度因子(0.01) */
    uint16_t res3;  /** 保留 */
    int32_t headVeh;  /** 车辆行驶方向(2D，1e-5) */
};

/** 北斗星历 */
struct PephBDS
{
    uint8_t res;  /** 保留 */
    uint8_t svid;  /** 卫星ID */
    uint32_t sqrtA;  /** 长半轴(2^{-19}) */
    uint32_t e;  /** 第一偏心率(2^{-33}) */
    int32_t M0;  /** 平均近地点角(2^{-31}pi, rad) */
    int16_t DeltaN;  /** 平均运动校正(2^{-43}pi, rad) */
    uint16_t toe;  /** 星历基准时间(2^4) */
    int32_t i0;     /** 倾角(2^{-31}pi, rad) */
    int16_t iDot;   /** 倾角变化率(2^{-43}pi/s, rad/s) */
    int32_t Omega0;  /** 周历升交点经度(2^{-31}pi, rad) */
    int32_t OmegaDot;  /** 赤经率(2^{-43}pi/s, rad/s) */
    int32_t w;  /** 近地点幅角(2^{-31}pi, rad) */
    int16_t Cuc;  /** ICD中的校正系数(2^{-29}) */
    int16_t Cus;  /** ICD中的校正系数(2^{-29}) */
    int16_t Crc;  /** ICD中的校正系数(2^{-5}) */
    int16_t Crs;  /** ICD中的校正系数(2^{-5}) */
    int16_t Cic;  /** ICD中的校正系数(2^{-29}) */
    int16_t Cis;  /** ICD中的校正系数(2^{-29}) */
    uint16_t toc;  /** 时钟基准时间(2^4) */
    int32_t af0;  /** SV时钟校正项0(2^{-33}) */
    int32_t af1;  /** SV时钟校正项1(2^{-50}) */
    int16_t af2;  /** SV时钟校正项2(2^{-66}) */
    int16_t tGD;  /** 群延迟(2^{-31}) */
    int8_t Alpha0;  /** 垂直延迟的振幅系数(2^{-30}) */
    int8_t Alpha1;  /** 垂直延迟的振幅系数(2^{-27}) */
    int8_t Alpha2;  /** 垂直延迟的振幅系数(2^{-24}) */
    int8_t Alpha3;  /** 垂直延迟的振幅系数(2^{-24}) */
    int8_t Beta0;  /** 模式周期系数(2^{11}) */
    int8_t Beta1;  /** 模式周期系数(2^{14}) */
    int8_t Beta2;  /** 模式周期系数(2^{15}) */
    int8_t Beta3;  /** 模式周期系数(2^{16}) */

    uint16_t weeknum;  /** 基准周数 TODO: 确认下这个是否是int16_t， 应该是 */
    uint8_t IODC;  /** 时钟数据版本号 */
    uint8_t IODE;  /** 星历数据版本号 */
    uint8_t ura;  /** 用户范围精度 */
    uint8_t health;  /** 使用状态 */
};

/** 北斗历书 */
struct PalmBDS
{
    uint8_t svid;  /** 卫星ID */
    uint8_t toa;  /** 历书时间(2^12) */
    uint16_t health;  /** 运行不佳的标志：0-不可操作，1-可操作（注：EPH和ALM中的0和1代表意义相反 */
    uint32_t sqrtA;  /** 长半轴(2^{-11}) */
    uint32_t e;  /** 第一偏心率(2^{-21}) */
    int32_t Omege0;  /** 周历升交点经度(2^{-23}pi, rad) */
    int32_t OmegeDot;  /** 赤经率(2^{-38}pi/s, rad/s) */
    int32_t w;  /** 近地点幅角(2^{-23}pi, rad) */
    int16_t di;  /** 倾角修正(2^{-19}pi, rad) */
    int16_t af0;  /** SV时钟校正项0(2^{-20}) */
    int16_t af1;  /** SV时钟校正项1(2^{-38}) */
    uint8_t WN;  /** 周数 */
};

};  // namespace F2B0

#endif /* _F2B0_TYPE_H_ */