#ifndef CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_2D_HPP
#define CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_ndt_2d/dynamic_maps/occupancy_gridmap.hpp>

namespace cslibs_mapping {
namespace maps {
template <typename T>
class OccupancyNDTGridMap2D : public Map
{
public:
    using Ptr      = std::shared_ptr<OccupancyNDTGridMap2D<T>>;
    using ConstPtr = std::shared_ptr<const OccupancyNDTGridMap2D<T>>;

    using map_t    =
    cslibs_ndt::map::Map<cslibs_ndt::map::tags::dynamic_map,2,
    cslibs_ndt::OccupancyDistribution,T,cslibs_indexed_storage::backend::kdtree::KDTree>;
    //cslibs_ndt_2d::dynamic_maps::OccupancyGridmap<T>;
    template <typename ... args_t>
    inline OccupancyNDTGridMap2D(const std::string &frame,
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

#endif // CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_2D_HPP
