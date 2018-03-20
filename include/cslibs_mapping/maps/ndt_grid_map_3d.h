#ifndef CSLIBS_MAPPING_NDT_GRID_MAP_3D_H
#define CSLIBS_MAPPING_NDT_GRID_MAP_3D_H

#include <cslibs_mapping/maps/map.h>
#include <cslibs_ndt_3d/dynamic_maps/gridmap.hpp>

namespace cslibs_mapping {
namespace maps {
class NDTGridMap3D : public Map
{
public:
    using Ptr      = std::shared_ptr<NDTGridMap3D>;
    using ConstPtr = std::shared_ptr<const NDTGridMap3D>;

    using map_t = cslibs_ndt_3d::dynamic_maps::Gridmap;
    template <typename ... args_t>
    NDTGridMap3D(const args_t &...args) :
        map_(new map_t(args...))
    {
    }

    map_t::Ptr getMap()
    {
        return map_;
    }

private:
    const map_t::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAP_3D_H
