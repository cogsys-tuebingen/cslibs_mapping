#ifndef CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H
#define CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/ndt_grid_map_3d.h>

namespace cslibs_mapping {
namespace mapper {
class NDTGridMapper3D : public Mapper
{
public:
    const inline map_t::ConstPtr getMap() const override;

private:
    virtual inline bool setupMap(ros::NodeHandle &nh) override;
    virtual inline bool uses(const data_t::ConstPtr &type) override;
    virtual inline void process(const data_t::ConstPtr &data) override;
    virtual inline bool saveMap() override;

    maps::NDTGridMap3D::Ptr         map_;
    mutable std::mutex              map_mutex_;
    mutable std::condition_variable map_notify_;
};
}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H
