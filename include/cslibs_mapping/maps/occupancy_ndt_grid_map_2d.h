#ifndef CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_2D_H
#define CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_2D_H

#include <cslibs_mapping/maps/map.h>
#include <cslibs_ndt_2d/dynamic_maps/occupancy_gridmap.hpp>

namespace cslibs_mapping {
namespace maps {
class OccupancyNDTGridMap2D : public Map
{
public:
    using Ptr      = std::shared_ptr<OccupancyNDTGridMap2D>;
    using ConstPtr = std::shared_ptr<const OccupancyNDTGridMap2D>;

    using map_t    = cslibs_ndt_2d::dynamic_maps::OccupancyGridmap;
    template <typename ... args_t>
    OccupancyNDTGridMap2D(const std::string &frame,
                          const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    map_t::Ptr getMap() const
    {
        return map_;
    }

private:
    const map_t::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_2D_H
