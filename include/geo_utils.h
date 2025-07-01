
#ifndef _GEO_UTILS_H_
#define _GEO_UTILS_H_

#include "eigen_defs.h"

class GeoUtils
{
public:
    static Vec3d llh2xyz(const Vec3d& llh);
    static Vec3d xyz2llh(const Vec3d& xyz);
    static Vec3d ecef2ned(const Vec3d &ref_ecef, const Vec3d &ecef);
    static Vec3d ned2ecef(const Vec3d &ref_ecef, const Vec3d &ned);
    static Vec3d llh2ned(const Vec3d &ref_llh, const Vec3d &llh);
    static Vec3d ned2llh(const Vec3d &ref_llh, const Vec3d &ned);
};


#endif