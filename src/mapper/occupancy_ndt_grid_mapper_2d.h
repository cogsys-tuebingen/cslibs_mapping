#ifndef CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_2D_H
#define CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_2D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_2d.hpp>

namespace cslibs_mapping {
namespace mapper {
class OccupancyNDTGridMapper2D : public Mapper
{
public:
    virtual const inline map_t::ConstPtr getMap() const override;

protected:
    inline void setupVisibilityBasedUpdateParameters(ros::NodeHandle &nh);
    virtual inline bool setupMap(ros::NodeHandle &nh) override;
    virtual inline bool uses(const data_t::ConstPtr &type) override;
    virtual inline void process(const data_t::ConstPtr &data) override;
    virtual inline bool saveMap() override;

private:
    maps::OccupancyNDTGridMap2D::Ptr map_;

    bool visibility_based_update_;
    cslibs_gridmaps::utility::InverseModel::Ptr ivm_;
    cslibs_gridmaps::utility::InverseModel::Ptr ivm_visibility_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_2D_H
