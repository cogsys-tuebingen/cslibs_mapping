#include <cslibs_mapping/mapper/ndt_grid_mapper_2d.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>
#include <cslibs_mapping/mapper/save_map.hpp>

namespace cslibs_mapping {
NDTGridMapper2d::NDTGridMapper2d(const double resolution,
                             const double sampling_resolution,
                             const std::string &frame_id) :
    stop_(false),
    request_map_(false),
    callback_([](const static_map_t::Ptr &){}),
    resolution_(resolution),
    sampling_resolution_(sampling_resolution),
    frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});
}

NDTGridMapper2d::~NDTGridMapper2d()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}


void NDTGridMapper2d::insert(const measurement_t &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void NDTGridMapper2d::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}

void NDTGridMapper2d::requestMap()
{
    request_map_ = true;
}

void NDTGridMapper2d::setCallback(const callback_t &cb)
{
    if(!request_map_) {
        callback_ = cb;
    }
}

void NDTGridMapper2d::loop()
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

void NDTGridMapper2d::mapRequest()
{
    if(request_map_ && dynamic_map_) {
        cslibs_ndt_2d::conversion::from(dynamic_map_, static_map_.data(), sampling_resolution_);
        cslibs_gridmaps::static_maps::algorithms::normalize<double>(*static_map_.data());
        callback_(static_map_);
    }
    request_map_ = false;
    notify_static_map_.notify_all();
}

void NDTGridMapper2d::process(const measurement_t &m)
{
    if(!dynamic_map_) {
        dynamic_map_.reset(new dynamic_map_t(cslibs_math_2d::Transform2d::identity(),
                                             resolution_));
        latest_time_ = m.stamp;
    }

    if(m.stamp > latest_time_) {
        latest_time_ = m.stamp;
    }

    cslibs_time::Time start = cslibs_time::Time::now();
    for(const auto &p : *(m.points)) {
        const cslibs_math_2d::Point2d pm = m.origin * p;
        dynamic_map_->add(pm);
    }
    std::cout << "[NDTGridMapper2d]: Insertion took " << (cslibs_time::Time::now() - start).milliseconds() << "ms \n";
}

bool NDTGridMapper2d::saveMap(
    const std::string    & path,
    const nav_msgs::Path & poses_path)
{
    std::cout << "[NDTGridMapper2d]: Saving Map..." << std::endl;
    while (q_.hasElements()) {
        request_map_ = true;
        lock_t static_map_lock(static_map_mutex_);
        notify_event_.notify_one();
        notify_static_map_.wait(static_map_lock);
    }

    if (!dynamic_map_) {
        std::cout << "[NDTGridMapper2d]: No Map." << std::endl;
        return true;
    }

    boost::filesystem::path p(path);

    if(!boost::filesystem::is_directory(p))
        boost::filesystem::create_directories(p);
    if(!boost::filesystem::is_directory(p)) {
        std::cout << "[NDTGridMapper2d]: '" << path << "' is not a directory." << std::endl;
        return false;
    }

    // save dynamic map
    cslibs_ndt_2d::dynamic_maps::saveBinary(dynamic_map_, (p / boost::filesystem::path("map")).string());

    if (!static_map_.data())
        return false;

    // save static map (occ.map.yaml, occ.map.pgm, occ.map.raw.pgm, poses.yaml)
    std::string occ_path_yaml    = (p / boost::filesystem::path("occ.map.yaml")).    string();
    std::string occ_path_pgm     = (p / boost::filesystem::path("occ.map.pgm")).     string();
    std::string occ_path_raw_yaml= (p / boost::filesystem::path("occ.map.raw.yaml")).string();
    std::string occ_path_raw_pgm = (p / boost::filesystem::path("occ.map.raw.pgm")). string();
    std::string poses_path_yaml  = (p / boost::filesystem::path("poses.yaml")).      string();

    if (cslibs_mapping::serialization::saveMap(occ_path_yaml, occ_path_pgm, "occ.map.pgm",
                                               occ_path_raw_yaml, occ_path_raw_pgm, "occ.map.raw.pgm",
                                               poses_path_yaml, poses_path,
                                               static_map_.data()->getData(), static_map_.data()->getHeight(),
                                               static_map_.data()->getWidth(), static_map_.data()->getOrigin(),
                                               static_map_.data()->getResolution())) {

        std::cout << "[NDTGridMapper2d]: Saved Map successfully." << std::endl;
        return true;
    }
    return false;
}
}
