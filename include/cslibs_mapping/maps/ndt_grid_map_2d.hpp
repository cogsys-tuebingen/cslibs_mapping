#ifndef CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP
#define CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_mapping/maps/map_template.hpp>
#include <cslibs_ndt_2d/dynamic_maps/gridmap.hpp>

namespace cslibs_mapping {
namespace maps {

template <cslibs_ndt::map::tags::option option_t = cslibs_ndt::map::tags::dynamic_map,
          typename T = double,
          template <typename, typename, typename...> class backend_t = cis::backend::simple::UnorderedMap>
using NDTGridMap2D =
cslibs_mapping::maps::MapTemplate<cslibs_ndt::map::Map<option_t,2,cslibs_ndt::Distribution,T,backend_t>>;

}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAP_2D_HPP
