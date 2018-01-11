#include <cslibs_mapping/mapper/octomap_mapper_3d.h>

//#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_mapping/mapper/save_map.hpp>
#include <cslibs_time/conversion/ros.hpp>

namespace cslibs_mapping {
OctomapMapper3d::OctomapMapper3d(
        const cslibs_gridmaps::utility::InverseModel &inverse_model,
        const double                                  resolution,
        const std::string                            &frame_id) :
        stop_(false),
        request_map_(false),
        callback_([](const static_map_t::Ptr &){}),
        inverse_model_(inverse_model),
        resolution_(resolution),
        frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});
}

OctomapMapper3d::~OctomapMapper3d()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}

void OctomapMapper3d::insert(const measurement_t &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void OctomapMapper3d::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}

void OctomapMapper3d::requestMap()
{
    request_map_ = true;
}

void OctomapMapper3d::setCallback(
        const callback_t & cb)
{
    if(!request_map_)
        callback_ = cb;
}

void OctomapMapper3d::loop()
{
    lock_t notify_event_mutex_lock(notify_event_mutex_);
    while(!stop_) {
        notify_event_.wait(notify_event_mutex_lock);
        while(q_.hasElements()) {
            if(stop_)
                break;

            //mapRequest();
            auto m = q_.pop();
            process(m);
        }
        mapRequest();
    }
}

void OctomapMapper3d::mapRequest()
{
    cslibs_time::Time now = cslibs_time::Time::now();

    if(request_map_ && dynamic_map_) {
        if (!static_map_.data())
            static_map_.data().reset(new static_map_t());

        static_map_.stamp() = latest_time_;
        static_map_.data()->header.frame_id = frame_id_;
        static_map_.data()->header.stamp    = cslibs_time::from(latest_time_);

        dynamic_map_->updateInnerOccupancy();
        octomap_msgs::fullMapToMsg(*dynamic_map_, *(static_map_.data()));

        callback_(static_map_);
        std::cout << "[OctomapMapper3d]: Serialization took " << (cslibs_time::Time::now() - now).milliseconds() << "ms \n";
    }
    request_map_ = false;
    notify_static_map_.notify_all();
}

void OctomapMapper3d::process(const measurement_t &m)
{
    if (!dynamic_map_) {
        dynamic_map_.reset(new dynamic_map_t(resolution_));
        dynamic_map_->setProbHit(inverse_model_.getProbOccupied());
        dynamic_map_->setProbMiss(inverse_model_.getProbFree());
        latest_time_ = m.stamp;
    }

    if (m.stamp > latest_time_)
        latest_time_ = m.stamp;

    cslibs_time::Time now = cslibs_time::Time::now();
    octomap::Pointcloud octomap_cloud;
    for (const auto & p : *(m.points))
        if (p.isNormal())
            octomap_cloud.push_back(p(0), p(1), p(2));
    const octomath::Vector3 origin(m.origin.translation()(0),
                                   m.origin.translation()(1),
                                   m.origin.translation()(2));
    dynamic_map_->insertPointCloud(octomap_cloud, origin, -1, true, true);
    std::cout << "[OctomapMapper3d]: Insertion took " << (cslibs_time::Time::now() - now).milliseconds() << "ms \n";
}

bool OctomapMapper3d::saveMap(
    const std::string    & path,
    const nav_msgs::Path & poses_path)
{
    std::cout << "[OctomapMapper3d]: Saving Map..." << std::endl;
    while (q_.hasElements()) {
        request_map_ = true;
        lock_t static_map_lock(static_map_mutex_);
        notify_event_.notify_one();
        notify_static_map_.wait(static_map_lock);
    }

    if (!dynamic_map_) {
        std::cout << "[OctomapMapper3d]: No Map." << std::endl;
        return true;
    }

    boost::filesystem::path p(path);

    if(!boost::filesystem::is_directory(p))
        boost::filesystem::create_directories(p);
    if(!boost::filesystem::is_directory(p)) {
        std::cout << "[OctomapMapper3d]: '" << path << "' is not a directory." << std::endl;
        return false;
    }

    std::cout << "[OctomapMapper3d]: Saved Map successful." << std::endl;
    return true;
}
}
