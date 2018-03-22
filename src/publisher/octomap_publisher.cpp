#include "octomap_publisher.h"

#include <cslibs_mapping/maps/occupancy_grid_map_3d.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::publisher::OctomapPublisher, cslibs_mapping::publisher::Publisher)

namespace cslibs_mapping {
namespace publisher {
bool OctomapPublisher::uses(const map_t::ConstPtr &map) const
{
    return map->isType<cslibs_mapping::maps::OccupancyGridMap3D>();
}

void OctomapPublisher::doAdvertise(ros::NodeHandle &nh, const std::string &topic)
{
    publisher_ = nh.advertise<octomap_msgs::Octomap>(topic, 1);
}

void OctomapPublisher::publish(const map_t::ConstPtr &map, const ros::Time &time)
{
    if (map->isType<cslibs_mapping::maps::OccupancyGridMap3D>()) {
        using local_map_t = octomap::OcTree;
        const std::shared_ptr<local_map_t> &m = map->as<cslibs_mapping::maps::OccupancyGridMap3D>().getMap();

        octomap_msgs::Octomap msg;
        octomap_msgs::fullMapToMsg(*m, msg);

        msg.header.stamp    = time;
        msg.header.frame_id = map->getFrame();

        publisher_.publish(msg);
    } else
        std::cout << "[OctomapPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
}
}
}
