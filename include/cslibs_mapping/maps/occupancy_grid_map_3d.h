#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_H
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_H

#include <cslibs_mapping/maps/map.h>
#include <octomap/OcTree.h>

namespace cslibs_mapping {
namespace maps {
class OccupancyGridMap3D : public Map
{
public:
    using Ptr      = std::shared_ptr<OccupancyGridMap3D>;
    using ConstPtr = std::shared_ptr<const OccupancyGridMap3D>;

    using map_t    = octomap::OcTree;
    template <typename ... args_t>
    OccupancyGridMap3D(const args_t &...args) :
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

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_H
