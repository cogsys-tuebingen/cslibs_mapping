#ifndef CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H
#define CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

namespace cslibs_mapping {
namespace publisher {
class PointcloudPublisher : public Publisher
{
private:
    virtual inline bool uses(const map_t::ConstPtr &map) const;
    virtual inline void doAdvertise(ros::NodeHandle &nh, const std::string &topic);
    virtual inline void publish(const map_t::ConstPtr &map, const ros::Time &time);

    inline void publishNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time);
    inline void publishOccupancyNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time);

    bool fast_;
    double occ_threshold_;
    cslibs_gridmaps::utility::InverseModel::Ptr ivm_;
};
}
}

#endif // CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H
