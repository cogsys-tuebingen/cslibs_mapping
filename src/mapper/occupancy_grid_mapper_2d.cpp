#include <cslibs_mapping/mapper/occupancy_grid_mapper_2d.h>

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_math/common/array.hpp>

#include <cslibs_gridmaps/serialization/dynamic_maps/probability_gridmap.hpp>
#include <cslibs_mapping/mapper/save_map.hpp>

namespace cslibs_mapping {
OccupancyGridMapper2d::OccupancyGridMapper2d(
        const cslibs_gridmaps::utility::InverseModel &inverse_model,
        const double                                  resolution,
        const double                                  chunk_resolution,
        const std::string                            &frame_id) :
    stop_(false),
    request_map_(false),
    callback_([](const static_map_t::Ptr &){}),
    inverse_model_(inverse_model),
    resolution_(resolution),
    chunk_resolution_(chunk_resolution),
    frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});

}

OccupancyGridMapper2d::~OccupancyGridMapper2d()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}


void OccupancyGridMapper2d::insert(const measurement_t &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void OccupancyGridMapper2d::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}

void OccupancyGridMapper2d::requestMap()
{
    request_map_ = true;
    notify_event_.notify_one();
}

void OccupancyGridMapper2d::setCallback(const callback_t &cb)
{
    if(!request_map_) {
        callback_ = cb;
    }
}

void OccupancyGridMapper2d::loop()
{
    lock_t notify_event_mutex_lock(notify_event_mutex_);
    while(!stop_) {
        notify_event_.wait(notify_event_mutex_lock);
        while(q_.hasElements()) {
            if(stop_)
                break;

            mapRequest();

            auto m = q_.pop();
            process(m);
        }
        mapRequest();
    }
}

void OccupancyGridMapper2d::mapRequest()
{
    if(request_map_ && dynamic_map_) {
        cslibs_math_2d::Transform2d origin = dynamic_map_->getOrigin();
        static_map_.data().reset(new static_map_t   (origin,
                                                     dynamic_map_->getResolution(),
                                                     dynamic_map_->getHeight(),
                                                     dynamic_map_->getWidth(),
                                                     cslibs_math::common::LogOdds::to(0.5)));

        static_map_.stamp() = latest_time_;

        const std::size_t chunk_step = dynamic_map_->getChunkSize();
        const dynamic_map_t::index_t min_chunk_index = dynamic_map_->getMinChunkIndex();
        const dynamic_map_t::index_t max_chunk_index = dynamic_map_->getMaxChunkIndex();
        for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++i) {
            for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++j) {
                dynamic_map_t::chunk_t *chunk = dynamic_map_->getChunk({{j,i}});
                if(chunk != nullptr) {
                    const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                    const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));
                    for(std::size_t k = 0 ; k < chunk_step ; ++k) {
                        for(std::size_t l = 0 ; l < chunk_step ; ++l) {
                            static_map_.data()->at(cx + l, cy + k) = chunk->at(l,k);
                        }
                    }
                }
            }
        }


        cslibs_gridmaps::static_maps::conversion::LogOdds::from(static_map_, static_map_);
        callback_(static_map_);
    }
    request_map_ = false;
    notify_static_map_.notify_one();
}

void OccupancyGridMapper2d::process(const measurement_t &m)
{
    if(!dynamic_map_) {
        dynamic_map_.reset(new dynamic_map_t(cslibs_math_2d::Transform2d::identity(),
                                             resolution_,
                                             chunk_resolution_,
                                             inverse_model_.getLogOddsPrior()));
        latest_time_ = m.stamp;
    }

    if(m.stamp > latest_time_) {
        latest_time_ = m.stamp;
    }

    cslibs_time::Time start = cslibs_time::Time::now();
    const double resolution2 = (resolution_ * resolution_ * 0.25);
    for(const auto &p : *(m.points)) {
        if(p.isNormal()) {
            const cslibs_math_2d::Point2d end_point = m.origin * p;
            auto b = dynamic_map_->getLineIterator(m.origin.translation(),
                                                   end_point);

            while(!b.done()) {
                double l = b.distance2() > resolution2 ? inverse_model_.updateFree(*b) : inverse_model_.updateOccupied(*b);
                *b = l;
                ++b;
            }
            *b = inverse_model_.updateOccupied(*b);
        }
    }
    std::cout << "[OccupancyGridMapper2d]: Insertion took " << (cslibs_time::Time::now() - start).milliseconds() << "ms \n";
}

bool OccupancyGridMapper2d::saveMap(
    const std::string    & path,
    const nav_msgs::Path & poses_path)
{
    if (!dynamic_map_)
        return false;

    boost::filesystem::path p(path);

    if(!boost::filesystem::is_directory(p))
        boost::filesystem::create_directories(p);
    if(!boost::filesystem::is_directory(p)) {
        std::cout << "[NDTGridMapper2d]: '" << path << "' is not a directory." << std::endl;
        return false;
    }

    // save dynamic map (YAML::Node)
    std::string map_path_yaml    = (p / boost::filesystem::path("map.yaml")).string();
    {
        std::ofstream map_out_yaml(map_path_yaml);
        if(!map_out_yaml.is_open()) {
          std::cout << "[NDTGridMapper2d]: Could not open file '" << map_path_yaml << "'." << std::endl;
          return false;
        }
        map_out_yaml << YAML::Node(dynamic_map_);
        map_out_yaml.close();
    }

    if (!static_map_.data())
        return false;

    // save static map (occ.map.yaml, occ.map.pgm, occ.map.raw.pgm, poses.yaml)
    std::string occ_path_yaml    = (p / boost::filesystem::path("occ.map.yaml")).   string();
    std::string occ_path_pgm     = (p / boost::filesystem::path("occ.map.pgm")).    string();
    std::string occ_path_raw_pgm = (p / boost::filesystem::path("occ.map.raw.pgm")).string();
    std::string poses_path_yaml  = (p / boost::filesystem::path("poses.yaml")).     string();

    return cslibs_mapping::saveMap(occ_path_yaml, occ_path_pgm, occ_path_raw_pgm, poses_path_yaml, poses_path,
                                   static_map_.data()->getData(), static_map_.data()->getHeight(),
                                   static_map_.data()->getWidth(), static_map_.data()->getOrigin(),
                                   static_map_.data()->getResolution());
}
}
