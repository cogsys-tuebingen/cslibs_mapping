#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_HPP
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_mapping/maps/map_template.hpp>
#include <octomap/OcTree.h>

namespace cslibs_mapping {
namespace maps {

using OccupancyGridMap3D =
cslibs_mapping::maps::MapTemplate<octomap::OcTree>;

}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_3D_HPP
