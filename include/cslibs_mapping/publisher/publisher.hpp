#ifndef CSLIBS_MAPPING_PUBLISHER_HPP
#define CSLIBS_MAPPING_PUBLISHER_HPP

#include <ros/ros.h>

#include <cslibs_mapping/mapper/mapper.hpp>

#include <cslibs_utility/signals/signals.hpp>
#include <cslibs_utility/common/delegate.hpp>

namespace cslibs_mapping {
namespace publisher {
class Publisher
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
        map_request_.disable();

        if (thread_.joinable())
            thread_.join();
    }

    inline const static std::string Type()
    {
        return "cslibs_mapping::publisher::Publisher";
    }

    inline std::string getName() const
    {
        return name_;
    }

    inline void setup(const std::map<std::string, mapper_t::Ptr> &mappers,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        std::string mapper_name = nh.getParam<std::string>(param_name("mapper"), "");
        if (mapper_name == "")
            throw std::runtime_error("[Publisher '" << name_ << "']: No mapper was found!");

        auto mapper = mappers.find(mapper_name);
        if (mapper == mappers.end())
            throw std::runtime_error("[Publisher '" << name_ << "']: Cannot find mapper '" + mapper_name + "'!");

        if (this->uses(mapper->getMap())) {
            // set mapper
            mapper_ = mapper;
            std::cout << "[Publisher '" << name_ << "']: Using mapper '" << mapper_name << "'." << std::endl;

            // advertise topic
            std::string topic = nh.getParam<std::string>(param_name("topic"), "");
            doAdvertise(nh, topic);
        }
    }

    inline void start()
    {
        thread_ = std::thread([this](){ loop(); });
    }

private:
    inline void loop()
    {
        ros::Rate r(publish_rate_);
        while (ros::ok()) {
            const ros::Time now = ros::Time::now();
            if (mapper_)
                if (const map_t::ConstPtr map = mapper_->getMap())
                    publish(map, now);

            r.sleep();
        }
    }

    inline void doAdvertise(ros::NodeHandle &nh, const std::string &topic) = 0;
    inline void publish(const map_t::ConstPtr &map, const ros::Time &time) = 0;

    std::thread    thread_;
    mapper_t::Ptr  mapper_;
    double         publish_rate_;

protected:
    std::string    name_;
    ros::Publisher publisher_;
};
}
}

#endif // CSLIBS_MAPPING_PUBLISHER_HPP
