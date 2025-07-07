
#ifndef _GINS_INTERFACE_H_
#define _GINS_INTERFACE_H_

#include "f2b0_type.h"

struct ImuData
{
    double timestamp;
    double acc[3];
    double gyr[3];
};

struct GnssData
{
    double timestamp;
    double blh[3];  /** latitude(deg), longitude(deg), height(m) */
    double dev[3];
    double vel_forward; /** m/s */
    double vel_track;   /** deg */
    double vel_upward;  /** m/s */
    int sat_num;
    int sol_type;
};

/** GNSS observation data by F2B0 */
struct GnssObsF2B0
{
    F2B0::NavTime nav_time;    
    F2B0::NavTimeUTC nav_time_utc;
    F2B0::NavClock nav_clock;
    F2B0::NavClock2 nav_clock2;
    F2B0::NavSvInfo nav_sv_info;
    F2B0::NavSvState nav_sv_state;
    F2B0::NavPVT nav_pvt;
};



#endif /* _GINS_INTERFACE_H_ */