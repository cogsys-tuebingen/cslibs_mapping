#ifndef CSLIBS_MAPPING_NDT_GRID_MAP_3D_H
#define CSLIBS_MAPPING_NDT_GRID_MAP_3D_H

#include <cslibs_mapping/maps/map.h>
#include <cslibs_ndt_3d/dynamic_maps/gridmap.hpp>
#include <cslibs_utility/synchronized/wrap_around.hpp>

namespace cslibs_mapping {
namespace maps {
class NDTGridMap3D : public Map
{
public:
    using Ptr      = std::shared_ptr<NDTGridMap3D>;
    using ConstPtr = std::shared_ptr<const NDTGridMap3D>;

    using mutex_t  = std::mutex;
    using lock_t   = std::unique_lock<mutex_t>;

    using map_t    = cslibs_ndt_3d::dynamic_maps::Gridmap;
    using handle_t = cslibs_utility::synchronized::WrapAround<const map_t::Ptr>;

    template <typename ... args_t>
    NDTGridMap3D(const std::string &frame,
                 const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    const inline handle_t get() const
    {
        return handle_t(&map_, &mutex_);
    }

private:
    mutable mutex_t  mutex_;
    const map_t::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAP_3D_H
