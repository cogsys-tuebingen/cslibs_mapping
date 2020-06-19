#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAPPER_3D_H
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAPPER_3D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/occupancy_grid_map_3d.hpp>

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>

#include <cslibs_time/time.hpp>
#include <cslibs_math/statistics/stable_distribution.hpp>

namespace cslibs_mapping {
namespace mapper {
class OccupancyGridMapper3D : public Mapper
{
public:
    virtual ~OccupancyGridMapper3D();
    virtual const inline map_t::ConstPtr getMap() const override;

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override;
    virtual inline bool uses(const data_t::ConstPtr &type) override;
    virtual inline bool process(const data_t::ConstPtr &data) override;

    template <typename T>
    inline bool doProcess(const data_t::ConstPtr &data)
    {
        const cslibs_plugins_data::types::Pointcloud3<T> &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud3<T>>();

        cslibs_math_3d::Transform3<T> o_T_d;
        if (tf_->lookupTransform(map_frame_,
                                 cloud_data.frame(),
                                 ros::Time().fromNSec(cloud_data.timeFrame().start.nanoseconds()),
                                 o_T_d,
                                 tf_timeout_)) {
            const typename cslibs_math_3d::Pointcloud3<T>::ConstPtr &points = cloud_data.points();
            if (points) {
                octomap::Pointcloud cloud;

                for (const auto &point : *points) {
                    if (point.isNormal()) {
                        const cslibs_math_3d::Point3<T> map_point = o_T_d * point;
                        if (map_point.isNormal())
                            cloud.push_back(map_point(0), map_point(1), map_point(2));
                    }
                }
                const octomath::Vector3 origin(o_T_d.translation()(0),
                                               o_T_d.translation()(1),
                                               o_T_d.translation()(2));

                const cslibs_time::Time start = cslibs_time::Time::now();
                map_->get()->insertPointCloud(cloud, origin, -1);
                const double time = (cslibs_time::Time::now() - start).milliseconds();
                stats_ += time;

                std::cout << "[OccupancyGridMapper3D]: N = " << stats_.getN() << std::endl;
                return true;
            }
        }
        return false;
    }

    virtual inline bool saveMap() override;

private:
    maps::OccupancyGridMap3D::Ptr map_;
    cslibs_math::statistics::StableDistribution<double,1,6> stats_;
    double iterations_;
    bool clear_;
};
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAPPER_3D_H
