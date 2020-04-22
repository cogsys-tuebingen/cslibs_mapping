#include "oru_publisher.h"

#include <cslibs_mapping/maps/oru_ndt_grid_map_3d.hpp>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::publisher::OruPublisher, cslibs_mapping::publisher::Publisher)

namespace cslibs_mapping {
namespace publisher {
bool OruPublisher::uses(const map_t::ConstPtr &map) const
{
    return map->isType<cslibs_mapping::maps::OruNDTGridMap3D>();
}

void OruPublisher::doAdvertise(ros::NodeHandle &nh, const std::string &topic)
{
    publisher_ = nh.advertise<ndt_map::NDTMapMsg>(topic, 1);
}

void OruPublisher::doPublish(const map_t::ConstPtr &map, const ros::Time &time)
{
    if (map->isType<cslibs_mapping::maps::OruNDTGridMap3D>()) {
        using local_map_t = perception_oru::NDTMap;
        const std::shared_ptr<local_map_t> m = map->as<cslibs_mapping::maps::OruNDTGridMap3D>().get();
        if (m) {

            ndt_map::NDTMapMsg msg;
            perception_oru::toMessage(m.get(), msg, map->getFrame());

            msg.header.stamp    = time;
            msg.header.frame_id = map->getFrame();

            publisher_.publish(msg);
            return;
        }
    }
    std::cout << "[OruPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
}
}
}
