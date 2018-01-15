#include <cslibs_mapping/mapper/ndt_grid_mapper_3d.h>
#include <cslibs_ndt_3d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_mapping/mapper/save_map.hpp>

namespace cslibs_mapping {
NDTGridMapper3d::NDTGridMapper3d(
        const double        resolution,
        const std::string & frame_id) :
        stop_(false),
        request_map_(false),
        callback_([](const static_map_t::Ptr &){}),
        resolution_(resolution),
        frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});
}

NDTGridMapper3d::~NDTGridMapper3d()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}

void NDTGridMapper3d::insert(const measurement_t &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void NDTGridMapper3d::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}

void NDTGridMapper3d::requestMap()
{
    request_map_ = true;
}

void NDTGridMapper3d::setCallback(
        const callback_t & cb)
{
    if(!request_map_)
        callback_ = cb;
}

void NDTGridMapper3d::setMarkerCallback(
        const marker_callback_t & cb)
{
    if(!request_map_)
        marker_callback_ = cb;
}

void NDTGridMapper3d::setDistributionsCallback(
        const distributions_callback_t & cb)
{
    if(!request_map_)
        distributions_callback_ = cb;
}

void NDTGridMapper3d::loop()
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

void NDTGridMapper3d::mapRequest()
{
    cslibs_time::Time now = cslibs_time::Time::now();
    cslibs_time::Duration dur_sampling;
    cslibs_time::Duration dur;

    if(request_map_ && dynamic_map_) {
        if (!static_map_.data())
            static_map_.data().reset(new static_map_t());
        if (!marker_map_)
            marker_map_.reset(new visualization_msgs::MarkerArray());
        if (!distributions_)
            distributions_.reset(new Distribution3dArray());

        marker_map_->markers.clear();
        static_map_.stamp() = latest_time_;
        distributions_->data.clear();
        distributions_->header.frame_id = frame_id_;
        distributions_->header.stamp = ros::Time::now();

        auto sample = [](const dynamic_map_t::distribution_t * d,
                         const point_t &p) {
            return d ? d->data().sampleNonNormalized(p) : 0.0;
        };
        auto sample_bundle = [&sample] (const dynamic_map_t::distribution_bundle_t * b,
                                        const point_t &p)
        {
            return 0.125 * (sample(b->at(0), p) +
                            sample(b->at(1), p) +
                            sample(b->at(2), p) +
                            sample(b->at(3), p) +
                            sample(b->at(4), p) +
                            sample(b->at(5), p) +
                            sample(b->at(6), p) +
                            sample(b->at(7), p));
        };

        for(auto &bi : updated_indices_) {
            cslibs_time::Time start_retrieve = cslibs_time::Time::now();
            const dynamic_map_t::distribution_bundle_t *b = dynamic_map_->getDistributionBundle(bi);
            dur += (cslibs_time::Time::now() - start_retrieve);
            if(b) {
                dynamic_map_t::distribution_t::distribution_t d;
                for (std::size_t i = 0; i < 8; ++ i)
                    d += b->at(i)->getHandle()->data();
                if (d.getN() == 0)
                    continue;

                point_t p(d.getMean());
                pcl::PointXYZI prob;
                prob.x = static_cast<float>(p(0));
                prob.y = static_cast<float>(p(1));
                prob.z = static_cast<float>(p(2));

                cslibs_time::Time start_sampling = cslibs_time::Time::now();
                prob.intensity = static_cast<float>(sample_bundle(b, p));
                dur_sampling += (cslibs_time::Time::now() - start_sampling);
                if (static_cast<int>(static_map_.data()->size()) <= b->id())
                    static_map_.data()->resize(b->id() + 1ul);
                static_map_.data()->points[b->id()] = prob;

                Distribution3d distr;
                distr.id.data = b->id();
                for (int i = 0; i < 3; ++ i) {
                    distr.mean[i].data          = p(i);
                    distr.eigen_values[i].data  = d.getEigenValues()(i);
                }
                for (int i = 0; i < 9; ++ i) {
                    distr.eigen_vectors[i].data = d.getEigenVectors()(i);
                    distr.covariance[i].data    = d.getCovariance()(i);
                }
                distr.prob.data = static_cast<double>(sample_bundle(b, p));
                distributions_->data.emplace_back(distr);

                drawMarker(b->id(), d, prob.intensity);
            }
        }
        updated_indices_.clear();

        callback_(static_map_);
        marker_callback_(marker_map_);
        distributions_callback_(distributions_);

        std::cout << "[NDTGridMapper3d]: Visualization [all]      took " << (cslibs_time::Time::now() - now).milliseconds() << "ms \n";
        std::cout << "[NDTGridMapper3d]: Visualization [retrieve] took " << dur.milliseconds() << "ms \n";
        std::cout << "[NDTGridMapper3d]: Visualization [sampling] took " << dur_sampling.milliseconds() << "ms \n";
    }
    request_map_ = false;
    notify_static_map_.notify_all();
}

void NDTGridMapper3d::drawMarker(
        const int                                           & id,
        const dynamic_map_t::distribution_t::distribution_t & d,
        const float                                         & prob)
{
    if (d.getN() < 3)
        return;

    visualization_msgs::Marker marker;
    marker.ns = "ndt";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // mean
    marker.pose.position.x = d.getMean()(0);
    marker.pose.position.y = d.getMean()(1);
    marker.pose.position.z = d.getMean()(2);

    // rotation matrix from eigenvectors
    Eigen::Matrix<double, 3, 3> cov = d.getEigenVectors();
    Eigen::Quaterniond quat(cov);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    // eigenvalues
    marker.scale.x = d.getEigenValues()(0) + 1e-3;
    marker.scale.y = d.getEigenValues()(1) + 1e-3;
    marker.scale.z = d.getEigenValues()(2) + 1e-3;

    marker.color.a = 1.0;

    // TODO
    marker.color.r = prob;
    marker.color.g = prob;
    marker.color.b = prob;

    if(std::isnormal(prob))
        marker_map_->markers.push_back(marker);
}

void NDTGridMapper3d::process(const measurement_t &m)
{
    if (!dynamic_map_) {
        dynamic_map_.reset(new dynamic_map_t(dynamic_map_t::transform_t::identity(),
                                             resolution_));
        latest_time_ = m.stamp;
    }

    if (m.stamp > latest_time_)
        latest_time_ = m.stamp;

    cslibs_time::Time now = cslibs_time::Time::now();
    for (const auto & p : *(m.points)) {
        const dynamic_map_t::point_t pm = m.origin * p;
        if (pm.isNormal()) {
            dynamic_map_t::index_t bi;
            dynamic_map_->add(pm, bi);
            updated_indices_.insert(bi);
        }
    }
    std::cout << "[NDTGridMapper3d]: Insertion took " << (cslibs_time::Time::now() - now).milliseconds() << "ms \n";
}

bool NDTGridMapper3d::saveMap(
    const std::string    & path,
    const nav_msgs::Path & poses_path)
{
    std::cout << "[NDTGridMapper3d]: Saving Map..." << std::endl;
    while (q_.hasElements()) {
        request_map_ = true;
        lock_t static_map_lock(static_map_mutex_);
        notify_event_.notify_one();
        notify_static_map_.wait(static_map_lock);
    }

    if (!dynamic_map_) {
        std::cout << "[NDTGridMapper3d]: No Map." << std::endl;
        return true;
    }

    boost::filesystem::path p(path);

    if(!boost::filesystem::is_directory(p))
        boost::filesystem::create_directories(p);
    if(!boost::filesystem::is_directory(p)) {
        std::cout << "[NDTGridMapper3d]: '" << path << "' is not a directory." << std::endl;
        return false;
    }

    // save dynamic map
    cslibs_ndt_3d::dynamic_maps::save(dynamic_map_, (p / boost::filesystem::path("map")).string());

    // save 3d path
    std::string poses_path_3d_yaml  = (p / boost::filesystem::path("poses3d.yaml")).     string();
    if (!cslibs_mapping::serialization::savePath(poses_path_3d_yaml, poses_path))
        return false;

    if (!static_map_.data())
        return false;
    // TODO: static map

    std::cout << "[NDTGridMapper3d]: Saved Map successful." << std::endl;
    return true;
}
}
