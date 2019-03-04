#ifndef CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP
#define CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_ndt_2d/dynamic_maps/gridmap.hpp>

namespace cslibs_mapping {
namespace maps {
template <typename T>
class NDTGridMap2D : public Map
{
public:
    using Ptr      = std::shared_ptr<NDTGridMap2D<T>>;
    using ConstPtr = std::shared_ptr<const NDTGridMap2D<T>>;

    using map_t    = cslibs_ndt_2d::dynamic_maps::Gridmap<T>;
    template <typename ... args_t>
    inline NDTGridMap2D(const std::string &frame,
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

#endif // CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP
