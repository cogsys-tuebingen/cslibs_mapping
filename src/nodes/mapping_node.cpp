#include "mapping_node.h"

#include <cslibs_plugins/plugin_loader.hpp>
#include <cslibs_plugins/plugin_factory.hpp>

namespace cslibs_mapping {
MappingNode::MappingNode() :
    nh_("~"),
    tf_(new tf_listener_t)
{
}

MappingNode::~MappingNode()
{
    for (auto &d : data_providers_)
        d.second->disable();
}

bool MappingNode::setup()
{
    {// load data providers
        cslibs_plugins::PluginLoader loader("cslibs_plugins_data", nh_);
        loader.load<data_provider_t, tf_listener_t::Ptr, ros::NodeHandle&>(
                    data_providers_, tf_, nh_);
        if (data_providers_.empty()) {
            ROS_ERROR_STREAM("No data provider was found!");
            return false;
        }

        std::string data_providers = "[";
        for (auto &d : data_providers_)
            data_providers += d.first + ",";
        data_providers.at(data_providers.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded data providers: " + data_providers);
    }

    cslibs_plugins::PluginLoader loader("cslibs_mapping", nh_);
    {// load publishers
        loader.load<publisher_t, ros::NodeHandle&>(publishers_, nh_);
          std::string publishers = "[";
          for (auto &p : publishers_)
              publishers += p.first + ",";
          publishers.at(publishers.size() - 1) = ']';
          ROS_INFO_STREAM("Loaded publishers: " + publishers);
    }

    {// load mappers
        loader.load<mapper_t, map_t<data_provider_t>, map_t<publisher_t>, ros::NodeHandle&>(
                    mappers_, data_providers_, publishers_, nh_);
        if (mappers_.empty()) {
            ROS_ERROR_STREAM("No mapper was found!");
            return false;
        }

        std::string mappers = "[";
        for (auto &m : mappers_)
            mappers += m.first + ",";
        mappers.at(mappers.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded mappers: " + mappers);
    }


    // advertise service
    service_ = nh_.advertiseService(nh_.getNamespace() + "/save_maps", &MappingNode::saveMaps, this);

    return true;
}

void MappingNode::start()
{
    for (auto &d : data_providers_)
        if (d.second)
            d.second->enable();

    for (auto &m : mappers_)
        if (m.second)
            m.second->start();

    const double node_rate = nh_.param<double>("node_rate", 60.0);
    if (node_rate == 0.0) {
        // unlimited speed
        ROS_INFO_STREAM("Spinning without rate!");
        ros::spin();
    } else {
        // limited speed
        ros::WallRate r(node_rate);
        ROS_INFO_STREAM("Spinning with " << node_rate << " Hz!");
        while (ros::ok()) {
            ros::spinOnce();
            r.sleep();
        }
    }
}

bool MappingNode::saveMaps(cslibs_mapping::SaveMap::Request &request, cslibs_mapping::SaveMap::Response &response)
{
    bool success = true;
    for (auto &m : mappers_)
        if (!m.second->saveMap(request.path.data))
            success = false;

    ROS_INFO_STREAM("Saved maps " << (success ? "successful!" : "unsuccessful!"));
    return success;
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cslibs_mapping");

    cslibs_mapping::MappingNode node;
    if (node.setup()) {
        ROS_INFO_STREAM("Node is set up and ready to start!");
        node.start();
    } else
        ROS_ERROR_STREAM("Cloud not set up the node!");

    return 0;
}
