#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_HPP
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <octomap/OcTree.h>

namespace cslibs_mapping {
namespace maps {
class OccupancyGridMap3D : public Map
{
public:
    using Ptr       = std::shared_ptr<OccupancyGridMap3D>;
    using ConstPtr  = std::shared_ptr<const OccupancyGridMap3D>;

    using map_t    = octomap::OcTree;
    template <typename ... args_t>
    inline OccupancyGridMap3D(const std::string &frame,
                              const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    const inline std::shared_ptr<map_t> get() const
    {
        return map_;
    }

private:
    const std::shared_ptr<map_t> map_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_HPP
