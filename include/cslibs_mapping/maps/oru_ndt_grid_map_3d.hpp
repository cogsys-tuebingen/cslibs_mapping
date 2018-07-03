#ifndef CSLIBS_MAPPING_ORU_NDT_GRID_MAP_3D_HPP
#define CSLIBS_MAPPING_ORU_NDT_GRID_MAP_3D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <ndt_map/ndt_map.h>

namespace cslibs_mapping {
namespace maps {
class OruNDTGridMap3D : public Map
{
public:
    using Ptr      = std::shared_ptr<OruNDTGridMap3D>;
    using ConstPtr = std::shared_ptr<const OruNDTGridMap3D>;

    using map_t    = lslgeneric::NDTMap;
    template <typename ... args_t>
    inline OruNDTGridMap3D(const std::string &frame,
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

#endif // CSLIBS_MAPPING_ORU_NDT_GRID_MAP_3D_HPP
