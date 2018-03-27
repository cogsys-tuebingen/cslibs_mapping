#ifndef CSLIBS_MAPPING_PUBLISHER_HPP
#define CSLIBS_MAPPING_PUBLISHER_HPP

#include <ros/ros.h>

#include <cslibs_mapping/mapper/mapper.hpp>

#include <cslibs_utility/signals/signals.hpp>
#include <cslibs_utility/common/delegate.hpp>

namespace cslibs_mapping {
namespace publisher {
class Publisher : public cslibs_plugins::Plugin
{
public:
    using Ptr          = std::shared_ptr<Publisher>;
    using mapper_t     = cslibs_mapping::mapper::Mapper;
    using map_t        = mapper_t::map_t;
    using request_t    = cslibs_utility::common::delegate<const map_t::ConstPtr&()>;
    using signal_t     = cslibs_utility::signals::Signal<request_t>;
    using connection_t = signal_t::Connection;

    inline Publisher() = default;
    inline ~Publisher()
    {
        if (thread_.joinable())
            thread_.join();
    }

    inline const static std::string Type()
    {
        return "cslibs_mapping::publisher::Publisher";
    }

    inline void setup(ros::NodeHandle &nh,
                      const std::map<std::string, mapper_t::Ptr> &mappers)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        publish_rate_ = nh.param<double>(param_name("rate"), 10.0);

        const std::string mapper_name = nh.param<std::string>(param_name("mapper"), "");
        if (mapper_name == "")
            throw std::runtime_error("[Publisher '" + name_ + "']: No mapper was found!");

        auto mapper = mappers.find(mapper_name);
        if (mapper == mappers.end())
            throw std::runtime_error("[Publisher '" + name_ + "']: Cannot find mapper '" + mapper_name + "'!");

        if (this->uses(mapper->second->getMap())) {
            // set mapper
            mapper_ = mapper->second;
            std::cout << "[Publisher '" << name_ << "']: Using mapper '" << mapper_name << "'." << std::endl;

            // advertise topic
            const std::string topic = nh.param<std::string>(param_name("topic"), "/cslibs_mapping/" + name_);
            doAdvertise(nh, topic);
        }
    }

    inline void start()
    {
        if (mapper_)
            thread_ = std::thread([this](){ loop(); });
    }

private:
    inline void loop()
    {
        assert (mapper_);

        ros::Rate r(publish_rate_);
        while (ros::ok()) {
            std::cout << "[Publisher '" << name_ << "']: Publishing ..." << std::endl;
            publish(mapper_->getMap(), ros::Time::now());
            r.sleep();
        }
    }

    virtual bool uses(const map_t::ConstPtr &map) const = 0;
    virtual void doAdvertise(ros::NodeHandle &nh, const std::string &topic) = 0;
    virtual void publish(const map_t::ConstPtr &map, const ros::Time &time) = 0;

    std::thread    thread_;
    mapper_t::Ptr  mapper_;
    double         publish_rate_;

protected:
    ros::Publisher publisher_;
};
}
}

#endif // CSLIBS_MAPPING_PUBLISHER_HPP
