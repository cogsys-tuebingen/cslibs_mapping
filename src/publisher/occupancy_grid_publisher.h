#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_PUBLISHER_H
#define CSLIBS_MAPPING_OCCUPANCY_GRID_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>

#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

namespace cslibs_mapping {
namespace publisher {
class OccupancyGridPublisher : public Publisher
{
private:
    virtual inline bool uses(const map_t::ConstPtr &map) const;
    virtual inline void doAdvertise(ros::NodeHandle &nh, const std::string &topic);
    virtual inline void doPublish(const map_t::ConstPtr &map, const ros::Time &time);

    inline void publishNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time);
    inline void publishOccupancyNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time);
    inline void publishOccupancyGridMap2D(const map_t::ConstPtr &map, const ros::Time &time);
    inline void publishMinMaxHeightMap2D(const map_t::ConstPtr &map, const ros::Time &time);
    inline void doPublish(const cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr &occ_map,
                        const ros::Time &time, const std::string &frame);

    double sampling_resolution_;
    cslibs_gridmaps::utility::InverseModel::Ptr ivm_;
    bool flattened_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_PUBLISHER_H
