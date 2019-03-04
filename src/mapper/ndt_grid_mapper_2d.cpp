#include "ndt_grid_mapper_2d.h"


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::NDTGridMapper2D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const NDTGridMapper2D::map_t::ConstPtr NDTGridMapper2D::getMap() const
{
    return map_;
}

bool NDTGridMapper2D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution = nh.param<double>(param_name("resolution"), 1.0);
    sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), (resolution / 40.0));
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
    assert (uses(data));

    const cslibs_plugins_data::types::Laserscan &laser_data = data->as<cslibs_plugins_data::types::Laserscan>();

    cslibs_math_2d::Transform2d o_T_d;
    if (tf_->lookupTransform(map_frame_,
                             laser_data.frame(),
                             ros::Time(laser_data.timeFrame().start.seconds()),
                             o_T_d,
                             tf_timeout_)) {

        const cslibs_plugins_data::types::Laserscan::rays_t rays = laser_data.getRays();
        cslibs_math_2d::Pointcloud2d::Ptr cloud(new cslibs_math_2d::Pointcloud2d);

        for (const auto &ray : rays)
            if (ray.valid() && ray.end_point.isNormal())
                cloud->insert(ray.end_point);

        map_->get()->insert(cloud, o_T_d);
    }
}

bool NDTGridMapper2D::saveMap()
{
    if (!map_) {
        std::cout << "[NDTGridMapper2D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[NDTGridMapper2D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[NDTGridMapper2D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }

    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr tmp;
    {
        if (!cslibs_ndt_2d::dynamic_maps::saveBinary(map_->get(), (path_ / boost::filesystem::path("map")).string()))
            return false;

        cslibs_ndt_2d::conversion::from(map_->get(), tmp, sampling_resolution_);
        if (!tmp)
            return false;
    }

    //cslibs_gridmaps::static_maps::algorithms::normalize<double>(*tmp);
    if (cslibs_mapping::mapper::saveMap(path_, nullptr, tmp->getData(), tmp->getHeight(),
                                        tmp->getWidth(), tmp->getOrigin(), tmp->getResolution())) {

        std::cout << "[NDTGridMapper2D '" << name_ << "']: Saved Map successfully." << std::endl;
        return true;
    }
    return false;
}
}
}
