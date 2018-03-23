#include "occupancy_grid_mapper_3d.h"

#include <cslibs_plugins_data/types/pointcloud.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::OccupancyGridMapper3D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const OccupancyGridMapper3D::map_t::ConstPtr OccupancyGridMapper3D::getMap() const
{
    map_->get().data()->updateInnerOccupancy();
    return map_;
}

bool OccupancyGridMapper3D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution = nh.param<double>(param_name("resolution"), 1.0);
    map_.reset(new maps::OccupancyGridMap3D(map_frame_, resolution));
    return true;
}

bool OccupancyGridMapper3D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Pointcloud>();
}

void OccupancyGridMapper3D::process(const data_t::ConstPtr &data)
{
    assert (uses(data));

    const cslibs_plugins_data::types::Pointcloud &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud>();

    tf::Transform o_T_d_tmp;
    if (tf_->lookupTransform(map_frame_,
                             cloud_data.getFrame(),
                             ros::Time(cloud_data.getTimeFrame().end.seconds()),
                             o_T_d_tmp,
                             tf_timeout_)) {
        cslibs_math_3d::Transform3d o_T_d = cslibs_math_ros::tf::conversion_3d::from(o_T_d_tmp);

        const cslibs_math_3d::Pointcloud3d::Ptr points = cloud_data.getPoints();
        if (points) {
            octomap::Pointcloud cloud;

            for (const auto &point : *points) {
                if (point.isNormal()) {
                    const cslibs_math_3d::Point3d map_point = o_T_d * point;
                    if (map_point.isNormal())
                        cloud.push_back(map_point(0), map_point(1), map_point(2));
                }
            }
            const octomath::Vector3 origin(o_T_d.translation()(0),
                                           o_T_d.translation()(1),
                                           o_T_d.translation()(2));
            const auto handle = map_->get();
            handle.data()->insertPointCloud(cloud, origin, -1, true, true);
        }
    }
}

bool OccupancyGridMapper3D::saveMap()
{
    if (!map_) {
        std::cout << "[OccupancyGridMapper3D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[OccupancyGridMapper3D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[OccupancyGridMapper3D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }

    std::string map_path_yaml = (path_ / boost::filesystem::path("map.yaml")).string();
    {
        std::ofstream map_out_yaml(map_path_yaml);
        if (!map_out_yaml.is_open()) {
            std::cout << "[OccupancyGridMapper3D '" << name_ << "']: Could not open file '" << map_path_yaml << "'." << std::endl;
            return false;
        }
        if (map_->get().data()->write(map_out_yaml)) {
            map_out_yaml.close();
            return true;
        }
        else {
            std::cout << "[OccupancyGridMapper3D '" << name_ << "']: Could not write to file '" << map_path_yaml << "'." << std::endl;
            return false;
        }
    }
    return true;
}
}
}
