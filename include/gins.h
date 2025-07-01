
#ifndef GINS_H
#define GINS_H

/**
 * @file gins.h
 * 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-30
 * 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include <stdint.h>

#include "eigen_defs.h"

#define MAX_SAT_NUM_BDS (64)    /** 最大卫星数 */
#define MAX_SIGNAL_NUM_BDS (5)  /** 单颗卫星最大信号数 */

struct NavInfo
{
    double timestamp;
    Vec3d posi;     /** enu frame */
    Vec3d velo;     /** enu frame */
    Quatd quat;     /** attitude */
    Vec3d accel;    /**  */ 

    Vec3d posi_dev;
    Vec3d velo_dev;
    Vec3d atti_dev;
    Vec3d accel_dev;
};

/** GNSS satellite information */
struct PephSatInfo
{
    uint8_t gnssid;  /** 卫星系统ID */
    uint8_t svid;    /** 卫星ID */
    Vec3d ecefPos;
    float pAcc;          /** 位置精度 */
    Vec3d ecefVel;
    float vAcc;          /** 速度精度 */
    double errClk;       /** 钟偏差 */
    double clkDrift;     /** 钟漂移 */
    double ionoDelay;   /** 离子时延 */
    double tropoDelay;   /** 透射时延 */
    double relativisticCorr;   /** 相对论纠正 */
};

#endif /* GINS_H */