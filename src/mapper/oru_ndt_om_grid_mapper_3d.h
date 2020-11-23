#ifndef CSLIBS_MAPPING_ORU_NDT_OM_GRID_MAPPER_3D_H
#define CSLIBS_MAPPING_ORU_NDT_OM_GRID_MAPPER_3D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/oru_ndt_grid_map_3d.hpp>

#include <cslibs_math/statistics/stable_distribution.hpp>

namespace cslibs_mapping {
namespace mapper {
class OruNDTOMGridMapper3D : public Mapper
{
public:
    virtual ~OruNDTOMGridMapper3D();
    virtual const inline map_t::ConstPtr getMap() const override;

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override;
    virtual inline bool uses(const data_t::ConstPtr &type) override;
    virtual inline bool process(const data_t::ConstPtr &data) override;
    virtual inline bool saveMap() override;

private:
    maps::OruNDTGridMap3D::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_ORU_NDT_OM_GRID_MAPPER_3D_H
