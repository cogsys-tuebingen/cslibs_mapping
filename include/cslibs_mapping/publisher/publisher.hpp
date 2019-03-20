#ifndef CSLIBS_MAPPING_PUBLISHER_HPP
#define CSLIBS_MAPPING_PUBLISHER_HPP

#include <ros/ros.h>

#include <cslibs_mapping/maps/map.hpp>

#include <cslibs_utility/signals/signals.hpp>
#include <cslibs_utility/common/delegate.hpp>
#include <cslibs_time/time.hpp>

#include <cslibs_plugins/plugin.hpp>

namespace cslibs_mapping {
namespace publisher {
class Publisher : public cslibs_plugins::Plugin
{
public:
    using Ptr          = std::shared_ptr<Publisher>;
    using map_t        = cslibs_mapping::maps::Map;
    using request_t    = cslibs_utility::common::delegate<const map_t::ConstPtr&()>;

    inline Publisher() = default;
    inline virtual ~Publisher() = default;

    inline const static std::string Type()
    {
        return "cslibs_mapping::publisher::Publisher";
    }

    inline void setup(ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        const double publish_rate = nh.param<double>(param_name("rate"), 10.0);
        const std::string topic = nh.param<std::string>(param_name("topic"), "/cslibs_mapping/" + name_);

        publish_period_ = cslibs_time::Duration(publish_rate > 0.0 ? 1.0 / publish_rate : 0.0);
        next_publish_   = cslibs_time::Time::now();
        doAdvertise(nh, topic);
    }

    inline void publish(const map_t::ConstPtr &map, const cslibs_time::Time &time)
    {
        cslibs_time::Time now = cslibs_time::Time::now();
        if (next_publish_ <= now) {
            doPublish(map, ros::Time::now());
            next_publish_ = now + publish_period_;
        }
    }

protected:
    virtual bool uses(const map_t::ConstPtr &map) const = 0;
    virtual void doAdvertise(ros::NodeHandle &nh, const std::string &topic) = 0;
    virtual void doPublish(const map_t::ConstPtr &map, const ros::Time &time) = 0;

    ros::Publisher          publisher_;
    cslibs_time::Time       next_publish_;
    cslibs_time::Duration   publish_period_;
};
}
}

#endif // CSLIBS_MAPPING_PUBLISHER_HPP
