#ifndef CSLIBS_MAPPING_ORU_NDT_GRID_MAP_3D_HPP
#define CSLIBS_MAPPING_ORU_NDT_GRID_MAP_3D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_mapping/maps/map_template.hpp>
#include <ndt_map/ndt_map.h>

namespace cslibs_mapping {
namespace maps {

using OruNDTGridMap3D =
cslibs_mapping::maps::MapTemplate<lslgeneric::NDTMap>;

}
}

#endif // CSLIBS_MAPPING_ORU_NDT_GRID_MAP_3D_HPP
