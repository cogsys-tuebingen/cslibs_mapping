#include <cslibs_mapping/mapper/occupancy_ndt_grid_mapper_2d.h>
#include <cslibs_math/common/log_odds.hpp>

#include <cslibs_ndt_2d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_mapping/mapper/save_map.hpp>

namespace cslibs_mapping {
OccupancyNDTGridMapper2d::OccupancyNDTGridMapper2d(
        const cslibs_gridmaps::utility::InverseModel &inverse_model,
        const double                                  resolution,
        const double                                  sampling_resolution,
        const std::string                            &frame_id) :
    stop_(false),
    request_map_(false),
    callback_([](const static_map_t::Ptr &){}),
    inverse_model_(inverse_model),
    resolution_(resolution),
    sampling_resolution_(sampling_resolution),
    frame_id_(frame_id)
{
    thread_ = std::thread([this](){loop();});
}

OccupancyNDTGridMapper2d::~OccupancyNDTGridMapper2d()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}


void OccupancyNDTGridMapper2d::insert(const measurement_t &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void OccupancyNDTGridMapper2d::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}

void OccupancyNDTGridMapper2d::requestMap()
{
    request_map_ = true;
}

void OccupancyNDTGridMapper2d::setCallback(const callback_t &cb)
{
    if(!request_map_) {
        callback_ = cb;
    }
}

void OccupancyNDTGridMapper2d::loop()
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

void OccupancyNDTGridMapper2d::mapRequest()
{
    if(request_map_ && dynamic_map_) {
        cslibs_math_2d::Transform2d origin = dynamic_map_->getOrigin();
        const std::size_t height = static_cast<std::size_t>(dynamic_map_->getHeight() / sampling_resolution_);
        const std::size_t width  = static_cast<std::size_t>(dynamic_map_->getWidth()  / sampling_resolution_);

        static_map_.data().reset(new static_map_t(origin,
                                                  sampling_resolution_,
                                                  height,
                                                  width));
        static_map_.stamp() = latest_time_;

        const double bundle_resolution = dynamic_map_->getBundleResolution();
        const int chunk_step = static_cast<int>(bundle_resolution / sampling_resolution_);
        const dynamic_map_t::index_t min_distribution_index = dynamic_map_->getMinDistributionIndex();
        const dynamic_map_t::index_t max_distribution_index = dynamic_map_->getMaxDistributionIndex();

        auto occupancy = [this] (const std::size_t& n_free, const std::size_t& n_occ) {
            return cslibs_math::common::LogOdds::from(
                        n_free * inverse_model_.getLogOddsFree() +
                        n_occ * inverse_model_.getLogOddsOccupied());
        };
        auto sample = [&occupancy] (const dynamic_map_t::distribution_t *d,
                                    const cslibs_math_2d::Point2d &p) {
            return (d && d->getDistribution()) ?
                        (d->getDistribution()->sampleNonNormalized(p) * occupancy(d->numFree(), d->numOccupied())) : 0.0;
        };
        auto sample_bundle = [&sample] (const dynamic_map_t::distribution_bundle_t* b,
                                        const cslibs_math_2d::Point2d &p)
        {
            return 0.25 * (sample(b->at(0), p) +
                           sample(b->at(1), p) +
                           sample(b->at(2), p) +
                           sample(b->at(3), p));
        };

        for(int i = min_distribution_index[1] ; i <= max_distribution_index[1] ; ++i) {
            for(int j = min_distribution_index[0] ; j <= max_distribution_index[0] ; ++j) {
                const dynamic_map_t::index_t bi = {{j,i}};
                dynamic_map_t::distribution_bundle_t* bundle   = dynamic_map_->getDistributionBundle(bi);
                if(bundle) {
                    const int cx = (j - min_distribution_index[0]) * static_cast<int>(chunk_step);
                    const int cy = (i - min_distribution_index[1]) * static_cast<int>(chunk_step);
                    for(int k = 0 ; k < chunk_step ; ++k) {
                        for(int l = 0 ; l < chunk_step ; ++l) {
                            const cslibs_math_2d::Point2d p(j * bundle_resolution + l * sampling_resolution_,
                                                            i * bundle_resolution + k * sampling_resolution_);
                            static_map_.data()->at(static_cast<std::size_t>(cx + l),
                                                   static_cast<std::size_t>(cy + k)) = sample_bundle(bundle, p);
                        }
                    }
                }
            }
        }
        cslibs_gridmaps::static_maps::algorithms::normalize<double>(*static_map_.data());
        callback_(static_map_);
    }
    request_map_ = false;
    notify_static_map_.notify_all();
}

void OccupancyNDTGridMapper2d::process(const measurement_t &m)
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
        dynamic_map_->add(m.origin.translation(), pm);
    }
    std::cout << "[OccupancyNDTGridMapper2d]: Insertion took " << (cslibs_time::Time::now() - start).milliseconds() << "ms \n";
}

bool OccupancyNDTGridMapper2d::saveMap(
    const std::string    & path,
    const nav_msgs::Path & poses_path)
{
    std::cout << "[OccupancyNDTGridMapper2d]: Saving Map..." << std::endl;
    while (q_.hasElements()) {
        request_map_ = true;
        lock_t static_map_lock(static_map_mutex_);
        notify_event_.notify_one();
        notify_static_map_.wait(static_map_lock);
    }

    if (!dynamic_map_) {
        std::cout << "[OccupancyNDTGridMapper2d]: No Map." << std::endl;
        return true;
    }

    boost::filesystem::path p(path);

    if(!boost::filesystem::is_directory(p))
        boost::filesystem::create_directories(p);
    if(!boost::filesystem::is_directory(p)) {
        std::cout << "[OccupancyNDTGridMapper2d]: '" << path << "' is not a directory." << std::endl;
        return false;
    }

    // save dynamic map (YAML::Node)
    std::string map_path_yaml    = (p / boost::filesystem::path("map.yaml")).string();
    {
        std::ofstream map_out_yaml(map_path_yaml);
        if(!map_out_yaml.is_open()) {
          std::cout << "[OccupancyNDTGridMapper2d]: Could not open file '" << map_path_yaml << "'." << std::endl;
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

    if (cslibs_mapping::serialization::saveMap(occ_path_yaml, occ_path_pgm, occ_path_raw_pgm, poses_path_yaml, poses_path,
                                               static_map_.data()->getData(), static_map_.data()->getHeight(),
                                               static_map_.data()->getWidth(), static_map_.data()->getOrigin(),
                                               static_map_.data()->getResolution())) {

        std::cout << "[OccupancyNDTGridMapper2d]: Saved Map successful." << std::endl;
        return true;
    }
    return false;
}
}
