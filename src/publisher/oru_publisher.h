#ifndef CSLIBS_MAPPING_ORU_PUBLISHER_H
#define CSLIBS_MAPPING_ORU_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>

namespace cslibs_mapping {
namespace publisher {
class OruPublisher : public Publisher
{
private:
    virtual inline bool uses(const map_t::ConstPtr &map) const;
    virtual inline void doAdvertise(ros::NodeHandle &nh, const std::string &topic);
    virtual inline void doPublish(const map_t::ConstPtr &map, const ros::Time &time);
};
}
}

#endif // CSLIBS_MAPPING_ORU_PUBLISHER_H
