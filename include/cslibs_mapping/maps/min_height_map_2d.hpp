#ifndef CSLIBS_MAPPING_MIN_HEIGHT_MAP_2D_HPP
#define CSLIBS_MAPPING_MIN_HEIGHT_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_gridmaps/dynamic_maps/min_heightmap.h>

namespace cslibs_mapping {
namespace maps {
template <typename Tp = double, typename T = double>
class MinHeightMap2D : public Map
{
public:
    using Ptr      = std::shared_ptr<MinHeightMap2D<Tp,T>>;
    using ConstPtr = std::shared_ptr<const MinHeightMap2D<Tp,T>>;

    using map_t    = cslibs_gridmaps::dynamic_maps::MinHeightmap<Tp,T>;
    template <typename ... args_t>
    inline MinHeightMap2D(const std::string &frame,
                          const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    const inline typename map_t::Ptr get() const
    {
        return map_;
    }

private:
    const typename map_t::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_MIN_HEIGHT_MAP_2D_HPP
