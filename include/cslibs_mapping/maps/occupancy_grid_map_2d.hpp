#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_HPP
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_mapping/maps/map_template.hpp>
#include <cslibs_gridmaps/dynamic_maps/probability_gridmap.h>

namespace cslibs_mapping {
namespace maps {

template <typename Tp = double, typename T = double>
using OccupancyGridMap2D =
cslibs_mapping::maps::MapTemplate<cslibs_gridmaps::dynamic_maps::ProbabilityGridmap<Tp,T>>;

}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAP_2D_HPP
