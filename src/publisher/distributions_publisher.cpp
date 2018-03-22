#include "distributions_publisher.h"

#include <cslibs_mapping/maps/ndt_grid_map_3d.h>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.h>

#include <cslibs_ndt_3d/conversion/distributions.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::publisher::DistributionsPublisher, cslibs_mapping::publisher::Publisher)

namespace cslibs_mapping {
namespace publisher {
bool DistributionsPublisher::uses(const map_t::ConstPtr &map) const
{
    return map->isType<cslibs_mapping::maps::NDTGridMap3D>() ||
           map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D>();
}

void DistributionsPublisher::doAdvertise(ros::NodeHandle &nh, const std::string &topic)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    fast_ = nh.param<bool>(param_name("fast"), false);
    const bool occupancy_ndt = nh.param<bool>(param_name("occupancy_ndt"), false);
    if (occupancy_ndt) {
        const double prob_prior    = nh.param(param_name("prob_prior"),    0.5);
        const double prob_free     = nh.param(param_name("prob_free"),     0.45);
        const double prob_occupied = nh.param(param_name("prob_occupied"), 0.65);
        ivm_.reset(new cslibs_gridmaps::utility::InverseModel(
                       prob_prior, prob_free, prob_occupied));
    }

    publisher_ = nh.advertise<cslibs_ndt_3d::DistributionArray>(topic, 1);
}

void DistributionsPublisher::publish(const map_t::ConstPtr &map, const ros::Time &time)
{
    if (map->isType<cslibs_mapping::maps::NDTGridMap3D>())
        publishNDTGridMap3D(map, time);
    else if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D>())
        publishOccupancyNDTGridMap3D(map, time);
    else
        std::cout << "[DistributionsPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
}

void DistributionsPublisher::publishNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
{
    using local_map_t = cslibs_ndt_3d::dynamic_maps::Gridmap;
    const local_map_t::Ptr &m = map->as<cslibs_mapping::maps::NDTGridMap3D>().getMap();

    cslibs_ndt_3d::DistributionArray::Ptr distributions;
    cslibs_ndt_3d::conversion::from(m, distributions, fast_);

    if (distributions) {
        distributions->header.stamp    = time;
        distributions->header.frame_id = map->getFrame();

        publisher_.publish(distributions);
    }
}

void DistributionsPublisher::publishOccupancyNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
{
    using local_map_t = cslibs_ndt_3d::dynamic_maps::OccupancyGridmap;
    const local_map_t::Ptr &m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap3D>().getMap();

    cslibs_ndt_3d::DistributionArray::Ptr distributions;
    cslibs_ndt_3d::conversion::from(m, distributions, ivm_, fast_);

    if (distributions) {
        distributions->header.stamp    = time;
        distributions->header.frame_id = map->getFrame();

        publisher_.publish(distributions);
    }
}
}
}
