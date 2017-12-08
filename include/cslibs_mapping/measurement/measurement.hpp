#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

#include <cslibs_time/time.hpp>
#include <cslibs_math/linear/pointcloud.hpp>

namespace cslibs_mapping {
template<typename point_t, typename transform_t>
struct Measurement {
    const typename cslibs_math::linear::Pointcloud<point_t>::Ptr  points;
    const transform_t                                             origin;
    const cslibs_time::Time                                       stamp;

    inline explicit Measurement(const typename cslibs_math::linear::Pointcloud<point_t>::Ptr &points,
                                const transform_t       &origin,
                                const cslibs_time::Time &stamp) :
        points(points),
        origin(origin),
        stamp(stamp)
    {
    }
};
}


#endif // MEASUREMENT_HPP
