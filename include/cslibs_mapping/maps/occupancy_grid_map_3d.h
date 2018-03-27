#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_H
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_H

#include <cslibs_mapping/maps/map.h>
#include <octomap/OcTree.h>
//#include <cslibs_utility/synchronized/wrap_around.hpp>

namespace cslibs_mapping {
namespace maps {
class OccupancyGridMap3D : public Map
{
public:
    using Ptr       = std::shared_ptr<OccupancyGridMap3D>;
    using ConstPtr  = std::shared_ptr<const OccupancyGridMap3D>;

//    using mutex_t  = std::mutex;
//    using lock_t   = std::unique_lock<mutex_t>;

    using map_t    = octomap::OcTree;
//    using handle_t = cslibs_utility::synchronized::WrapAround<const std::shared_ptr<map_t>>;

    template <typename ... args_t>
    OccupancyGridMap3D(const std::string &frame,
                 const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    const inline std::shared_ptr<map_t> get() const
    {
        return map_;
    }
/*
    const inline handle_t get() const
    {
        return handle_t(&map_, &mutex_);
    }*/

private:
//    mutable mutex_t              mutex_;
    const std::shared_ptr<map_t> map_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_H
