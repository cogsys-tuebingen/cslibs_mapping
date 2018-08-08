#include "ndt_grid_mapper_3d.h"

#include <cslibs_plugins_data/types/pointcloud.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_ndt_3d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_time/time.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::NDTGridMapper3D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const NDTGridMapper3D::map_t::ConstPtr NDTGridMapper3D::getMap() const
{
    return map_;
}

bool NDTGridMapper3D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution = nh.param<double>(param_name("resolution"), 1.0);
    std::vector<double> origin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    origin = nh.param<std::vector<double>>(param_name("origin"), origin);

    if (origin.size() != 6)
        return false;

    map_.reset(new maps::NDTGridMap3D(
                   map_frame_,
                   cslibs_math_3d::Pose3d(cslibs_math_3d::Vector3d(origin[0], origin[1], origin[2]),
                                          cslibs_math_3d::Quaternion(origin[3], origin[4], origin[5])),
                   resolution));
    return true;
}

bool NDTGridMapper3D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Pointcloud>();
}

void NDTGridMapper3D::process(const data_t::ConstPtr &data)
{
    assert (uses(data));

    const cslibs_plugins_data::types::Pointcloud &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud>();

    tf::Transform o_T_d_tmp;
    if (tf_->lookupTransform(map_frame_,
                             cloud_data.getFrame(),
                             ros::Time(cloud_data.getTimeFrame().start.seconds()),
                             o_T_d_tmp,
                             tf_timeout_)) {
        cslibs_math_3d::Transform3d o_T_d = cslibs_math_ros::tf::conversion_3d::from(o_T_d_tmp);
        if (const cslibs_math_3d::Pointcloud3d::Ptr &cloud = cloud_data.getPoints())
            map_->get()->insert(cloud, o_T_d);
    }
}

bool NDTGridMapper3D::saveMap()
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

    if (cslibs_ndt_3d::dynamic_maps::saveBinary(map_->get(), (path_ / boost::filesystem::path("map")).string())) {
        std::cout << "[NDTGridMapper3D '" << name_ << "']: Saved Map successfully." << std::endl;
        return true;
    }
    return false;
}
}
}
