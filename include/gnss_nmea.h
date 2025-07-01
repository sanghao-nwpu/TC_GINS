#ifndef _GINS_GNSS_SOL_H_
#define _GINS_GNSS_SOL_H_

#include "eigen_defs.h"
#include "geo_utils.h"
#include "interface.h"
#include "f2b0_type.h"
#include "gins.h"

/**
 * @brief Gnss的解算结果
 * 
 */
class GnssNmea
{
public:
    GnssNmea() = default;
    GnssNmea(const GnssData &data) 
        : timestamp_(data.timestamp),
          blh_(data.blh[0], data.blh[1], data.blh[2]),
          dev_(data.dev[0], data.dev[1], data.dev[2]),
          vel_forward_(data.vel_forward),
          vel_track_(data.vel_track),
          vel_upward_(data.vel_upward),
          sat_num_(data.sat_num),
          sol_type_(data.sol_type) {}

    const Vec3d &blh() const { return blh_; }
    const Vec3d &dev() const { return dev_; }
    double timestamp() const { return timestamp_; }
    double vel_forward() const { return vel_forward_; }
    double vel_track() const { return vel_track_; }
    double vel_upward() const { return vel_upward_; }
    int sat_num() const { return sat_num_; }
    int sol_type() const { return sol_type_; }

    Vec3d pos(const Vec3d &ref_llh) const 
    {
        return GeoUtils::llh2ned(ref_llh, blh_);
    }

    Vec3d vel() const 
    {
        if (std::isnan(vel_forward_) || std::isnan(vel_track_)) {
            throw std::runtime_error("Invalid velocity forward or track angle");
        }
        Vec3d result;
        double sin_track = sin(vel_track_);
        double cos_track = cos(vel_track_);
        result(0) = vel_forward_ * sin_track;
        result(1) = vel_forward_ * cos_track;
        result(2) = vel_upward_;
        return result;
    }

private:
    double timestamp_;
    Vec3d blh_;
    Vec3d dev_;
    double vel_forward_;
    double vel_track_;
    double vel_upward_;
    int sat_num_;
    int sol_type_;
};

#endif /* _GINS_GNSS_H_ */
