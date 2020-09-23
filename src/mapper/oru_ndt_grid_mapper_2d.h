#ifndef CSLIBS_MAPPING_ORU_NDT_GRID_MAPPER_2D_H
#define CSLIBS_MAPPING_ORU_NDT_GRID_MAPPER_2D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/oru_ndt_grid_map_3d.hpp>

#include <cslibs_math/statistics/stable_distribution.hpp>

namespace cslibs_mapping {
namespace mapper {
class OruNDTGridMapper2D : public Mapper
{
public:
    virtual ~OruNDTGridMapper2D();
    virtual const inline map_t::ConstPtr getMap() const override;

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override;
    virtual inline bool uses(const data_t::ConstPtr &type) override;
    virtual inline bool process(const data_t::ConstPtr &data) override;
    virtual inline bool saveMap() override;

private:
    maps::OruNDTGridMap3D::Ptr map_;
    cslibs_math::statistics::StableDistribution<double,1,6> stats_;
    double iterations_;
    bool clear_;

    double ndt_oru_local_size_x_;
    double ndt_oru_local_size_y_;
    double ndt_oru_local_size_z_;

    double varz_;
};
}
}

#endif // CSLIBS_MAPPING_ORU_NDT_GRID_MAPPER_2D_H
