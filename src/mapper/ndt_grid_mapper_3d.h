#ifndef CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H
#define CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/ndt_grid_map_3d.hpp>

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_ndt_3d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_time/time.hpp>

namespace cslibs_mapping {
namespace mapper {
template <typename T = double>
class NDTGridMapper3DBase : public Mapper
{
public:
    using rep_t = maps::NDTGridMap3D<T>;
    virtual const inline map_t::ConstPtr getMap() const override
    {
        return map_;
    }

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const T resolution = static_cast<T>(nh.param<double>(param_name("resolution"), 1.0));
        std::vector<double> origin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        origin = nh.param<std::vector<double>>(param_name("origin"), origin);

        if (origin.size() != 6)
            return false;

        map_.reset(new rep_t(
                       map_frame_,
                       cslibs_math_3d::Pose3<T>(
                            cslibs_math_3d::Vector3<T>(static_cast<T>(origin[0]), static_cast<T>(origin[1]), static_cast<T>(origin[2])),
                            cslibs_math_3d::Quaternion<T>(static_cast<T>(origin[3]), static_cast<T>(origin[4]), static_cast<T>(origin[5]))),
                       resolution));
        return true;
    }

    virtual inline bool uses(const data_t::ConstPtr &type) override
    {
        return type->isType<cslibs_plugins_data::types::Pointcloud3<T>>();
    }

    virtual inline void process(const data_t::ConstPtr &data) override
    {
        assert (uses(data));

        const cslibs_plugins_data::types::Pointcloud3<T> &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud3<T>>();

        cslibs_math_3d::Transform3<T> o_T_d;
        if (tf_->lookupTransform(map_frame_,
                                 cloud_data.frame(),
                                 ros::Time(cloud_data.timeFrame().start.seconds()),
                                 o_T_d,
                                 tf_timeout_)) {
            if (const typename cslibs_math_3d::Pointcloud3<T>::ConstPtr &cloud = cloud_data.points())
                map_->get()->insert(cloud, o_T_d);
        }
    }

    virtual inline bool saveMap() override
    {
        if (!map_) {
            std::cout << "[NDTGridMapper3D '" << name_ << "']: No Map." << std::endl;
            return true;
        }

        std::cout << "[NDTGridMapper3D '" << name_ << "']: Saving Map..." << std::endl;
        if (!checkPath()) {
            std::cout << "[NDTGridMapper3D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
            return false;
        }

        using path_t = boost::filesystem::path;
        path_t path_root(path_);
        if (!cslibs_ndt::common::serialization::create_directory(path_root))
            return false;

        if (cslibs_ndt_3d::dynamic_maps::saveBinary<T>(map_->get(), (path_ / boost::filesystem::path("map")).string())) {
            std::cout << "[NDTGridMapper3D '" << name_ << "']: Saved Map successfully." << std::endl;
            return true;
        }
        return false;
    }

private:
    typename rep_t::Ptr map_;
};

using NDTGridMapper3D   = NDTGridMapper3DBase<double>;
using NDTGridMapper3D_d = NDTGridMapper3DBase<double>;
using NDTGridMapper3D_f = NDTGridMapper3DBase<float>;
}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H
