#ifndef CSLIBS_MAPPING_MAPPER_HPP
#define CSLIBS_MAPPING_MAPPER_HPP

#include <thread>
#include <ros/ros.h>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data_provider.hpp>

#include <cslibs_utility/synchronized/synchronized_queue.hpp>
#include <cslibs_math_ros/tf/tf_listener.hpp>

#include <cslibs_mapping/maps/map.hpp>
#include <cslibs_mapping/mapper/save_map.hpp>
#include <cslibs_mapping/publisher/publisher.hpp>

#include <condition_variable>

namespace cslibs_mapping {
namespace mapper {
class Mapper : public cslibs_plugins::Plugin
{
public:
    using Ptr             = std::shared_ptr<Mapper>;
    using data_t          = cslibs_plugins_data::Data;
    using data_provider_t = cslibs_plugins_data::DataProvider;
    using publisher_t     = cslibs_mapping::publisher::Publisher;
    using map_t           = cslibs_mapping::maps::Map;
    using tf_listener_t   = cslibs_math_ros::tf::TFListener;
    using handle_vector_t = std::vector<typename data_provider_t::connection_t::Ptr>;
    using data_queue_t    = cslibs_utility::synchronized::queue<data_t::ConstPtr>;

    using mutex_t = std::mutex;
    using cond_t  = std::condition_variable;
    using lock_t  = std::unique_lock<mutex_t>;

    inline Mapper() = default;
    virtual ~Mapper()
    {
        stop_ = true;
        notify_.notify_one();

        while (queue_.hasElements())
            queue_.pop();

        if (thread_.joinable())
            thread_.join();
    }

    inline const static std::string Type()
    {
        return "cslibs_mapping::mapper::Mapper";
    }

    inline void setup(const std::map<std::string, data_provider_t::Ptr> &data_providers,
                      const std::map<std::string, typename publisher_t::Ptr> &publishers,
                      const tf_listener_t::Ptr &tf_listener, ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        auto callback = [this](const data_t::ConstPtr &data) {
            if (this->uses(data)) {
                queue_.emplace(data);
                notify_.notify_one();
            }
        };

        // map frame
        map_frame_  = nh.param<std::string>(param_name("map_frame"), "/map");
        queue_size_ = static_cast<std::size_t>(nh.param<int>(param_name("queue_size"), 10));

        // tf listener and lookup timeout
        //tf_.reset(new tf_listener_t);
        tf_ = tf_listener;
        tf_timeout_ = ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));

        // publication parameters
        force_publish_  = nh.param<bool>(param_name("force_publish"), false);
        double rate = nh.param<double>(param_name("publish_rate"), 10.0);
        publish_period_ = cslibs_time::Duration(rate == 0.0 ? 0.0 : (1.0 / rate));
        publish_number_ = nh.param<int>(param_name("publish_every_n"), std::numeric_limits<int>::infinity());

        /// retrieve data providers
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
        std::cout << "[Mapper '" << name_ << "']: Using data providers '" << ds << "'." << "\n";

        /// retrieve map publishers
        std::vector<std::string> publisher_names;
        nh.getParam(param_name("map_publishers"), publisher_names);

        if(publisher_names.empty()) {
            std::cerr << "[Mapper '" + name_ + "']: Using no publishers! \n";
        } else {
          std::string ps = "[";
          for (auto p : publisher_names) {
              if (publishers.find(p) == publishers.end())
                  throw std::runtime_error("[Mapper '" + name_ + "']: Cannot find publisher '" + p + "'!");

              publishers_.emplace_back(publishers.at(p));
              ps += p + ",";
          }
          ps.back() = ']';
          std::cout << "[Mapper '" << name_ << "']: Using publishers '" << ps << "'." << "\n";
        }

        // initialize map
        if (!setupMap(nh))
             throw std::runtime_error("[Mapper '" + name_ + "']: Map could not be initialized!");
    }

    inline void start()
    {
        stop_   = false;
        thread_ = std::thread([this](){ loop(); });
    }

    inline bool saveMap(const std::string &path)
    {
        path_ = (boost::filesystem::path(path) / boost::filesystem::path(name_)).string();
        return saveMap();
    }

    virtual const map_t::ConstPtr getMap() const = 0;

protected:
    std::vector<typename publisher_t::Ptr>  publishers_;

    std::thread        thread_;
    bool               stop_;

    handle_vector_t    handles_;
    data_queue_t       queue_;
    std::size_t        queue_size_;

    mutable mutex_t    mutex_;
    cond_t             notify_;

    std::string        map_frame_;
    std::string        path_;

    tf_listener_t::Ptr tf_;
    ros::Duration      tf_timeout_;

    cslibs_time::Duration publish_period_;
    int                   publish_number_;
    bool                  force_publish_;

    virtual inline void loop()
    {
        tf_.reset(new tf_listener_t);
        ros::Time::waitForValid();
        cslibs_time::Time pub = cslibs_time::Time::now();
        const std::chrono::nanoseconds pd(int(publish_period_.seconds() * 1e9));

        int n = 0;
        lock_t l(mutex_);
        while (!stop_) {
            if (queue_.empty())
                notify_.wait(l);//_for(l, pd);

            while (queue_.hasElements()) {
                if (stop_)
                    break;

                if (queue_size_ > 0) { // queue_size = 0 stands for infty
                    while (queue_.size() > queue_size_)
                        queue_.pop();
                }

                if (process(queue_.pop())) {
                    ++n;
                    cslibs_time::Time now = cslibs_time::Time::now();
                    // if publication is forced, check if timeout or n_out is reached
                    if (force_publish_ && (now >= pub || n >= publish_number_)) {
                        publish();
                        n = 0;
                        pub = now + publish_period_;
                    }
                }
            }

            // if publication is not forced, publish if there is time left
            if (queue_.empty() && !force_publish_)
                publish();
        }
    }

    virtual bool setupMap(ros::NodeHandle &nh) = 0;
    virtual bool uses(const data_t::ConstPtr &type) = 0;
    virtual bool process(const data_t::ConstPtr &data) = 0;
    virtual bool saveMap() = 0;

    virtual inline void publish()
    {
        for (auto &p : publishers_)
            p->publish(getMap(), cslibs_time::Time::now());
    }

    inline bool checkPath() const
    {
        if (!boost::filesystem::is_directory(path_))
            boost::filesystem::create_directories(path_);
        if (!boost::filesystem::is_directory(path_))
            return false;
        return true;
    }
};
}
}

#endif // CSLIBS_MAPPING_MAPPER_HPP
