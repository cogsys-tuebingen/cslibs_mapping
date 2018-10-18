#ifndef CSLIBS_MAPPING_MIN_MAX_HEIGHT_MAP_2D_HPP
#define CSLIBS_MAPPING_MIN_MAX_HEIGHT_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_gridmaps/dynamic_maps/min_max_heightmap.h>

namespace cslibs_mapping {
namespace maps {
class MinMaxHeightMap2D : public Map
{
public:
    using Ptr      = std::shared_ptr<MinMaxHeightMap2D>;
    using ConstPtr = std::shared_ptr<const MinMaxHeightMap2D>;

    using map_t    = cslibs_gridmaps::dynamic_maps::MinMaxHeightmap;
    template <typename ... args_t>
    inline MinMaxHeightMap2D(const std::string &frame,
                             const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    const inline map_t::Ptr get() const
    {
        return map_;
    }

private:
    const map_t::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_MIN_MAX_HEIGHT_MAP_2D_HPP
