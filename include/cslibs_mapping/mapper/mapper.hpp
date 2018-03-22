#ifndef CSLIBS_MAPPING_MAPPER_HPP
#define CSLIBS_MAPPING_MAPPER_HPP

#include <thread>
#include <ros/ros.h>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data_provider_2d.hpp>

#include <cslibs_utility/synchronized/synchronized_queue.hpp>
#include <cslibs_math_ros/tf/tf_listener_2d.hpp>

#include <cslibs_mapping/maps/map.h>
#include <cslibs_mapping/mapper/save_map.hpp>

namespace cslibs_mapping {
namespace mapper {
class Mapper : public cslibs_plugins::Plugin
{
public:
    using Ptr             = std::shared_ptr<Mapper>;
    using data_t          = cslibs_plugins_data::Data;
    using data_provider_t = cslibs_plugins_data::DataProvider2D;
    using map_t           = cslibs_mapping::maps::Map;
    using tf_listener_t   = cslibs_math_ros::tf::TFListener2d;

    inline Mapper() = default;
    inline ~Mapper()
    {
        stop_ = true;

        while (queue_.hasElements())
            queue_.pop();

        if (thread_.joinable())
            thread_.join();
    }

    inline const static std::string Type()
    {
        return "cslibs_mapping::mapper::Mapper";
    }

    inline void setup(const tf_listener_t::Ptr &tf,
                      ros::NodeHandle &nh,
                      const std::map<std::string, data_provider_t::Ptr> &data_providers)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        auto callback = [this](const data_t::ConstPtr &data) {
            if (this->uses(data))
                queue_.emplace(data);
        };

        map_frame_  = nh.param<std::string>(param_name("map_frame"), "/map");
        path_       = nh.param<std::string>(param_name("path"), "/tmp/maps");
        tf_         = tf;
        tf_timeout_ = ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));

        std::vector<std::string> data_provider_names;
        nh.getParam(param_name("data_providers"), data_provider_names);

        if (data_provider_names.empty())
            throw std::runtime_error("[Mapper '" + name_ + "']: No data providers were found!");

        std::string ds = "[";
        for (auto d : data_provider_names) {

            auto provider = data_providers.find(d);
            if (provider == data_providers.end())
                throw std::runtime_error("[Mapper '" + name_ + "']: Cannot find data provider '" + d + "'!");

            handles_.emplace_back(provider->second->connect(callback));
            ds += d + ",";
        }
        ds.back() = ']';
        std::cout << "[Mapper '" << name_ << "']: Using data providers '" << ds << "'." << std::endl;

        // initialize map
        if (!setupMap(nh))
             throw std::runtime_error("[Mapper '" + name_ + "']: Map could not be initialized!");
    }

    inline void start()
    {
        thread_ = std::thread([this](){ loop(); });
    }

    virtual const map_t::ConstPtr getMap() const = 0;
    virtual bool saveMap() = 0;

private:
    inline void loop()
    {
        while (!stop_) {
            if (queue_.hasElements())
                process(queue_.pop());
        }
    }

    virtual bool setupMap(ros::NodeHandle &nh) = 0;
    virtual bool uses(const data_t::ConstPtr &type) = 0;
    virtual void process(const data_t::ConstPtr &data) = 0;

    std::thread thread_;
    bool        stop_;

    std::vector<typename data_provider_t::connection_t::Ptr> handles_;
    cslibs_utility::synchronized::queue<data_t::ConstPtr>    queue_;

protected:
    inline bool checkPath() const
    {
        if (!boost::filesystem::is_directory(path_))
            boost::filesystem::create_directories(path_);
        if (!boost::filesystem::is_directory(path_))
            return false;
        return true;
    }

    std::string map_frame_;
    std::string path_;

    tf_listener_t::Ptr tf_;
    ros::Duration      tf_timeout_;
};
}
}

#endif // CSLIBS_MAPPING_MAPPER_HPP
