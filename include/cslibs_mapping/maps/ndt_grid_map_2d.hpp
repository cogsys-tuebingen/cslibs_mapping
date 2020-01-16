#ifndef CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP
#define CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_mapping/maps/map_template.hpp>
#include <cslibs_ndt_2d/dynamic_maps/gridmap.hpp>

namespace cslibs_mapping {
namespace maps {

template <typename T>
using NDTGridMap2D =
cslibs_mapping::maps::MapTemplate<cslibs_ndt_2d::dynamic_maps::Gridmap<T>>;

}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP
