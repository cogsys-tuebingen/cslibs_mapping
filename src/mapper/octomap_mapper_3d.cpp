#include <cslibs_mapping/mapper/octomap_mapper_3d.h>

#include <cslibs_mapping/mapper/save_map.hpp>
#include <cslibs_time/conversion/ros.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

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
    for (const auto & p : *(m.points)) {
        const auto pm = m.origin * p;
        if (pm.isNormal())
            octomap_cloud.push_back(pm(0), pm(1), pm(2));
    }
    const octomath::Vector3 origin(m.origin.translation()(0),
                                   m.origin.translation()(1),
                                   m.origin.translation()(2));
    dynamic_map_->insertPointCloud(octomap_cloud, origin, -1, true, true);
    const double time_ms = (cslibs_time::Time::now() - now).milliseconds();
    std::cout << "[OctomapMapper3d]: Insertion took " << time_ms << "ms \n";

    stats_ += time_ms;
    static const std::string filename = "/tmp/octomap_stats";
    std::ofstream out;
    out.open(filename, std::ofstream::out | std::ofstream::app);
    out << stats_.getN() << " | " << time_ms << " | " << stats_.getMean() << " | " << stats_.getStandardDeviation()
        << " | mem: " << dynamic_map_->memoryUsage() << std::endl;
    out.close();
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

    // save dynamic map (YAML::Node)
    std::string map_path_yaml    = (p / boost::filesystem::path("map.ot")).string();
    {
        std::ofstream map_out_yaml(map_path_yaml);
        if(!map_out_yaml.is_open()) {
          std::cout << "[OctomapMapper3d]: Could not open file '" << map_path_yaml << "'." << std::endl;
          return false;
        }
        dynamic_map_->write(map_out_yaml);
        map_out_yaml.close();
    }

    // save 3d path
    std::string poses_path_3d_yaml  = (p / boost::filesystem::path("poses3d.yaml")).     string();
    if (!cslibs_mapping::serialization::savePath(poses_path_3d_yaml, poses_path))
        return false;

    if (!static_map_.data())
        return false;

    // save static map (occ.map.yaml, occ.map.pgm, occ.map.raw.pgm, poses.yaml)
    std::string occ_path_yaml    = (p / boost::filesystem::path("occ.map.yaml")).    string();
    std::string occ_path_pgm     = (p / boost::filesystem::path("occ.map.pgm")).     string();
    std::string occ_path_raw_yaml= (p / boost::filesystem::path("occ.map.raw.yaml")).string();
    std::string occ_path_raw_pgm = (p / boost::filesystem::path("occ.map.raw.pgm")). string();
    std::string poses_path_yaml  = (p / boost::filesystem::path("poses.yaml")).      string();

    // convert octomap to probability gridmap
    const nav_msgs::OccupancyGrid::Ptr grid = toGrid(dynamic_map_);
    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr occ_map;
    cslibs_gridmaps::static_maps::conversion::from(grid, occ_map);

    if (cslibs_mapping::serialization::saveMap(occ_path_yaml, occ_path_pgm, "occ.map.pgm",
                                               occ_path_raw_yaml, occ_path_raw_pgm, "occ.map.raw.pgm",
                                               poses_path_yaml, poses_path,
                                               occ_map->getData(), occ_map->getHeight(),
                                               occ_map->getWidth(), occ_map->getOrigin(),
                                               occ_map->getResolution())) {
        std::cout << "[OctomapMapper3d]: Saved Map successfully." << std::endl;
        return true;
    }
    return false;
}

nav_msgs::OccupancyGrid::Ptr OctomapMapper3d::toGrid(const std::shared_ptr<dynamic_map_t> octomap)
{
    double minX, minY, minZ, maxX, maxY, maxZ;
    octomap->getMetricMin(minX, minY, minZ);
    octomap->getMetricMax(maxX, maxY, maxZ);
    octomap::point3d minPt = octomap::point3d(minX, minY, minZ);

    unsigned int tree_depth = octomap->getTreeDepth();
    octomap::OcTreeKey paddedMinKey = octomap->coordToKey(minPt);

    nav_msgs::OccupancyGrid::Ptr occupancy_map(new nav_msgs::OccupancyGrid);

    unsigned int width, height;
    double res;

    const std::size_t octree_depth = sizeof(unsigned short) * 8;
    unsigned int ds_shift = tree_depth - octree_depth;

    occupancy_map->info.resolution        = res = octomap->getNodeSize(octree_depth);
    occupancy_map->info.width             = width = (maxX-minX) / res + 1;
    occupancy_map->info.height            = height = (maxY-minY) / res + 1;
    occupancy_map->info.origin.position.x = minX  - (res / (float)(1 << ds_shift) ) + res;
    occupancy_map->info.origin.position.y = minY  - (res / (float)(1 << ds_shift) );

    occupancy_map->data.clear();
    occupancy_map->data.resize(width*height, -1);

    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(octree_depth, octomap->getTreeDepth());
    for (typename dynamic_map_t::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it) {
        bool occupied = octomap->isNodeOccupied(*it);
        int intSize = 1 << (octree_depth - it.getDepth());

        octomap::OcTreeKey minKey=it.getIndexKey();

        for (int dx = 0 ; dx < intSize ; ++dx) {
            for (int dy = 0 ; dy < intSize ; ++dy) {
                int posX = std::max<int>(0, minKey[0] + dx - paddedMinKey[0]);
                posX >>= ds_shift;

                int posY = std::max<int>(0, minKey[1] + dy - paddedMinKey[1]);
                posY >>= ds_shift;

                int idx = width * posY + posX;

                if (occupied)
                    occupancy_map->data[idx] = 100;
                else if (occupancy_map->data[idx] == -1)
                    occupancy_map->data[idx] = 0;
            }
        }
    }

    return occupancy_map;
}
}
