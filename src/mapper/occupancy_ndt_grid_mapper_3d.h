#ifndef CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_3D_H
#define CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_3D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.h>

namespace cslibs_mapping {
namespace mapper {
class OccupancyNDTGridMapper3D : public Mapper
{
public:
    const inline map_t::ConstPtr getMap() const override;

private:
    inline setupVisibilityBasedUpdateParameters(ros::NodeHandle &nh);
    virtual inline bool setupMap(ros::NodeHandle &nh) override;
    virtual inline bool uses(const data_t::ConstPtr &type) override;
    virtual inline void process(const data_t::ConstPtr &data) override;
    virtual inline bool saveMap() override;

    maps::OccupancyNDTGridMap3D::Ptr map_;
    mutable std::mutex               map_mutex_;
    mutable std::condition_variable  map_notify_;

    bool visibility_based_update_;
    cslibs_gridmaps::utility::InverseModel::Ptr ivm_;
    cslibs_gridmaps::utility::InverseModel::Ptr ivm_visibility_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_3D_H
