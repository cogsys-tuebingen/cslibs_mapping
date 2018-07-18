#include "occupancy_ndt_grid_mapper_3d.h"

#include <cslibs_plugins_data/types/pointcloud.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::OccupancyNDTGridMapper3D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const OccupancyNDTGridMapper3D::map_t::ConstPtr OccupancyNDTGridMapper3D::getMap() const
{
    return map_;
}

void OccupancyNDTGridMapper3D::setupVisibilityBasedUpdateParameters(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    visibility_based_update_ = nh.param<bool>(param_name("visibility_based_update"), false);
    if (!visibility_based_update_)
        return;

    const double prob_prior    = nh.param(param_name("prob_prior"),    0.5);
    const double prob_free     = nh.param(param_name("prob_free"),     0.45);
    const double prob_occupied = nh.param(param_name("prob_occupied"), 0.65);
    ivm_.reset(new cslibs_gridmaps::utility::InverseModel(
                   prob_prior, prob_free, prob_occupied));

    double visibility_threshold         = nh.param<double>(param_name("visibility_threshold"), 0.4);
    double prob_visible_if_occluded     = nh.param<double>(param_name("prob_visible_if_occluded"), 0.2);
    double prob_visible_if_not_occluded = nh.param<double>(param_name("prob_visible_if_not_occluded"), 0.8);
    ivm_visibility_.reset(new cslibs_gridmaps::utility::InverseModel(
                              visibility_threshold, prob_visible_if_occluded, prob_visible_if_not_occluded));

}

bool OccupancyNDTGridMapper3D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    double resolution = nh.param<double>(param_name("resolution"), 1.0);
    std::vector<double> origin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    origin = nh.param<std::vector<double>>(param_name("origin"), origin);

    if (origin.size() != 6)
        return false;

    setupVisibilityBasedUpdateParameters(nh);
    map_.reset(new maps::OccupancyNDTGridMap3D(
                   map_frame_,
                   cslibs_math_3d::Pose3d(cslibs_math_3d::Vector3d(origin[0], origin[1], origin[2]),
                                          cslibs_math_3d::Quaternion(origin[3], origin[4], origin[5])),
                   resolution));
    return true;
}

bool OccupancyNDTGridMapper3D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Pointcloud>();
}

void OccupancyNDTGridMapper3D::process(const data_t::ConstPtr &data)
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
        if (const cslibs_math_3d::Pointcloud3d::Ptr &cloud = cloud_data.getPoints()) {
            const cslibs_time::Time start = cslibs_time::Time::now();
            visibility_based_update_ ?
                        map_->get()->insertVisible(o_T_d, cloud, ivm_, ivm_visibility_) :
                        map_->get()->insert(o_T_d, cloud);
            const double time = (cslibs_time::Time::now() - start).milliseconds();
            stats_ += time;
            stats_print_ += "[OccupancyNDTGridMapper3D]: N | current | mean | std | mem = " +
                    std::to_string(stats_.getN()) + " | " + std::to_string(time)
                    + " | " + std::to_string(stats_.getMean())
                    + " | " + std::to_string(stats_.getStandardDeviation())
                    + " | " + std::to_string(map_->get()->getByteSize()) + "\n";
        }
    }
}

bool OccupancyNDTGridMapper3D::saveMap()
{
    if (!map_) {
        std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }    

    using path_t = boost::filesystem::path;
    path_t path_root(path_);
    if (!cslibs_ndt::common::serialization::create_directory(path_root))
        return false;

    std::ofstream out((path_root / path_t("stats")).string(), std::fstream::trunc);
    out << stats_print_ << std::endl;
    out.close();

    if (cslibs_ndt_3d::dynamic_maps::saveBinary(map_->get(), (path_ / boost::filesystem::path("map")).string())) {
        std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: Saved Map successfully." << std::endl;
        return true;
    }
    return false;
}
}
}
