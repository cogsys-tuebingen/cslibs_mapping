#ifndef CSLIBS_MAPPING_NODE_H
#define CSLIBS_MAPPING_NODE_H

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/publisher/publisher.hpp>
#include <cslibs_mapping/SaveMap.h>

#include <cslibs_plugins_data/data_provider_2d.hpp>

namespace cslibs_mapping {
class MappingNode
{
public:
    inline MappingNode();
    inline ~MappingNode();

    inline bool setup();
    inline void start();

private:
    inline bool saveMaps(cslibs_mapping::SaveMap::Request &request, cslibs_mapping::SaveMap::Response &response);

    template <typename T>
    using map_t           = std::map<std::string, typename T::Ptr>;
    using data_provider_t = cslibs_plugins_data::DataProvider2D;
    using mapper_t        = cslibs_mapping::mapper::Mapper;
    using publisher_t     = cslibs_mapping::publisher::Publisher;
    using tf_listener_t   = cslibs_math_ros::tf::TFListener2d;

    ros::NodeHandle        nh_;
    ros::ServiceServer     service_;
    tf_listener_t::Ptr     tf_;

    map_t<data_provider_t> data_providers_;
    map_t<mapper_t>        mappers_;
    map_t<publisher_t>     publishers_;
};
}

#endif // CSLIBS_MAPPING_NODE_H
