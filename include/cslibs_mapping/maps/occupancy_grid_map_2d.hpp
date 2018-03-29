#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_HPP
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_gridmaps/dynamic_maps/probability_gridmap.h>

namespace cslibs_mapping {
namespace maps {
class OccupancyGridMap2D : public Map
{
public:
    using Ptr      = std::shared_ptr<OccupancyGridMap2D>;
    using ConstPtr = std::shared_ptr<const OccupancyGridMap2D>;

    using map_t    = cslibs_gridmaps::dynamic_maps::ProbabilityGridmap;
    template <typename ... args_t>
    inline OccupancyGridMap2D(const std::string &frame,
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

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_HPP
