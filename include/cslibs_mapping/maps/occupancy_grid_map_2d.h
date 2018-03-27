#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_H
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_H

#include <cslibs_mapping/maps/map.h>
#include <cslibs_gridmaps/dynamic_maps/probability_gridmap.h>
//#include <cslibs_utility/synchronized/wrap_around.hpp>

namespace cslibs_mapping {
namespace maps {
class OccupancyGridMap2D : public Map
{
public:
    using Ptr      = std::shared_ptr<OccupancyGridMap2D>;
    using ConstPtr = std::shared_ptr<const OccupancyGridMap2D>;

//    using mutex_t  = std::mutex;
//    using lock_t   = std::unique_lock<mutex_t>;

    using map_t    = cslibs_gridmaps::dynamic_maps::ProbabilityGridmap;
//    using handle_t = cslibs_utility::synchronized::WrapAround<const map_t::Ptr>;

    template <typename ... args_t>
    OccupancyGridMap2D(const std::string &frame,
                 const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    const inline map_t::Ptr get() const
    {
        return map_;
    }
/*
    const inline handle_t get() const
    {
        return handle_t(&map_, &mutex_);
    }*/

private:
//    mutable mutex_t  mutex_;
    const map_t::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_H
