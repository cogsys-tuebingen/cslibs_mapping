#include "ndt_grid_mapper_2d.h"

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::NDTGridMapper2D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const NDTGridMapper2D::map_t::ConstPtr NDTGridMapper2D::getMap() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_)
        map_notify_.wait(l);

    return map_;
}

bool NDTGridMapper2D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution = nh.param<double>(param_name("resolution"), 1.0);
    std::vector<double> origin = {0.0, 0.0, 0.0};
    nh.param<std::vector<double>>(param_name("origin"), origin);

    if (origin.size() != 3)
        return false;

    map_.reset(new maps::NDTGridMap2D(
                   map_frame_,
                   cslibs_math_2d::Pose2d(origin[0], origin[1], origin[2]), resolution));
    return true;
}

bool NDTGridMapper2D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Laserscan>();
}

void NDTGridMapper2D::process(const data_t::ConstPtr &data)
{
    std::unique_lock<std::mutex> l(map_mutex_);
    const cslibs_plugins_data::types::Laserscan &laser_data = data->as<cslibs_plugins_data::types::Laserscan>();

    cslibs_math_2d::Transform2d o_T_d;
    if (tf_->lookupTransform(map_frame_,
                             laser_data.getFrame(),
                             ros::Time(laser_data.getTimeFrame().end.seconds()),
                             o_T_d,
                             tf_timeout_)) {

        const cslibs_plugins_data::types::Laserscan::rays_t rays = laser_data.getRays();
        cslibs_math_2d::Pointcloud2d::Ptr cloud(new cslibs_math_2d::Pointcloud2d);

        for (const auto &ray : rays) {
            if (ray.valid()) {
                const cslibs_math_2d::Point2d map_point = o_T_d * ray.point;
                if (map_point.isNormal())
                    cloud->insert(map_point);
            }
        }
        map_->getMap()->insert(o_T_d, cloud);
    }

    map_notify_.notify_all();
}

bool NDTGridMapper2D::saveMap()
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_) {
        std::cout << "[NDTGridMapper2D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[NDTGridMapper2D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[NDTGridMapper2D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }

    if (!cslibs_ndt_2d::dynamic_maps::saveBinary(map_->getMap(), (path_ / boost::filesystem::path("map")).string()))
        return false;

    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr tmp;
    cslibs_ndt_2d::conversion::from(map_->getMap(), tmp, map_->getMap()->getResolution() / 10.0);
    if (!tmp)
        return false;

    cslibs_gridmaps::static_maps::algorithms::normalize<double>(*tmp);
    if (cslibs_mapping::mapper::saveMap(path_, nullptr, tmp->getData(), tmp->getHeight(),
                                        tmp->getWidth(), tmp->getOrigin(), tmp->getResolution())) {

        std::cout << "[NDTGridMapper2D '" << name_ << "']: Saved Map successfully." << std::endl;
        return true;
    }
    return false;
}
}
}
