#ifndef CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_3D_HPP
#define CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_3D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_mapping/maps/map_template.hpp>
#include <cslibs_ndt_3d/dynamic_maps/occupancy_gridmap.hpp>

namespace cslibs_mapping {
namespace maps {

template <typename T>
using OccupancyNDTGridMap3D =
cslibs_mapping::maps::MapTemplate<cslibs_ndt_3d::dynamic_maps::OccupancyGridmap<T>>;

}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAP_3D_HPP
