#ifndef CSLIBS_MAPPING_MIN_MAX_HEIGHT_MAPPER_2D_H
#define CSLIBS_MAPPING_MIN_MAX_HEIGHT_MAPPER_2D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/min_max_height_map_2d.hpp>

namespace cslibs_mapping {
namespace mapper {
class MinMaxHeightMapper2D : public Mapper
{
public:
    virtual const inline map_t::ConstPtr getMap() const override;

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override;
    virtual inline bool uses(const data_t::ConstPtr &type) override;
    virtual inline void process(const data_t::ConstPtr &data) override;
    virtual inline bool saveMap() override;

private:
    maps::MinMaxHeightMap2D::Ptr map_;
};
}
}

#endif // CSLIBS_MAPPING_MIN_MAX_HEIGHT_MAPPER_2D_H
