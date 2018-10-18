#include "occupancy_grid_publisher.h"

#include <cslibs_mapping/maps/ndt_grid_map_2d.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_2d.hpp>
#include <cslibs_mapping/maps/occupancy_grid_map_2d.hpp>
#include <cslibs_mapping/maps/min_height_map_2d.hpp>
#include <cslibs_mapping/maps/distribution_height_map_2d.hpp>

#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>
#include <cslibs_ndt_2d/conversion/flatten.hpp>

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::publisher::OccupancyGridPublisher, cslibs_mapping::publisher::Publisher)

namespace cslibs_mapping {
namespace publisher {
bool OccupancyGridPublisher::uses(const map_t::ConstPtr &map) const
{
    return map->isType<cslibs_mapping::maps::NDTGridMap2D>() ||
           map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D>() ||
           map->isType<cslibs_mapping::maps::OccupancyGridMap2D>() ||
           map->isType<cslibs_mapping::maps::MinHeightMap2D>() ||
           map->isType<cslibs_mapping::maps::DistributionHeightMap2D>();
}

void OccupancyGridPublisher::doAdvertise(ros::NodeHandle &nh, const std::string &topic)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), 0.025);
    const bool occupancy = nh.param<bool>(param_name("occupancy"), false);
    if (occupancy) {
        const double prob_prior    = nh.param(param_name("prob_prior"),    0.5);
        const double prob_free     = nh.param(param_name("prob_free"),     0.45);
        const double prob_occupied = nh.param(param_name("prob_occupied"), 0.65);
        ivm_.reset(new cslibs_gridmaps::utility::InverseModel(
                       prob_prior, prob_free, prob_occupied));
    }
    flattened_ = nh.param<bool>(param_name("flatten"), false);
    if(flattened_) {
      ROS_WARN_STREAM("Publishing the flatttened map.");
    }

    publisher_ = nh.advertise<nav_msgs::OccupancyGrid>(topic, 1);
}

void OccupancyGridPublisher::doPublish(const map_t::ConstPtr &map, const ros::Time &time)
{
    if (map->isType<cslibs_mapping::maps::NDTGridMap2D>())
        return publishNDTGridMap2D(map, time);
    if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D>())
        return publishOccupancyNDTGridMap2D(map, time);
    if (map->isType<cslibs_mapping::maps::OccupancyGridMap2D>())
        return publishOccupancyGridMap2D(map, time);
    if (map->isType<cslibs_mapping::maps::MinHeightMap2D>())
        return publishMinHeightMap2D(map, time);
    if (map->isType<cslibs_mapping::maps::DistributionHeightMap2D>())
        return publishDistributionHeightMap2D(map, time);
    std::cout << "[OccupancyGridPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
}

void OccupancyGridPublisher::publishNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
{
    using local_map_t = cslibs_ndt_2d::dynamic_maps::Gridmap;
    const local_map_t::Ptr m = map->as<cslibs_mapping::maps::NDTGridMap2D>().get();
    if (m && !m->empty()) {
        cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr occ_map;

        if(flattened_) {
          std::cerr << "flattening" << std::endl;
          cslibs_ndt_2d::static_maps::flat::Gridmap::Ptr fm = cslibs_ndt_2d::conversion::flatten(m);
          std::cerr << "flattened" << std::endl;
          cslibs_ndt_2d::conversion::from(fm, occ_map, sampling_resolution_);
        } else {
          cslibs_ndt_2d::conversion::from(m, occ_map, sampling_resolution_);
        }
        if (occ_map) {
            doPublish(occ_map, time, map->getFrame());
            return;
        }
    }
    std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
}

void OccupancyGridPublisher::publishOccupancyNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
{
    if (ivm_) {
        using local_map_t = cslibs_ndt_2d::dynamic_maps::OccupancyGridmap;
        const local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap2D>().get();
        if (m && !m->empty()) {
            cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr occ_map;
            cslibs_ndt_2d::conversion::from(m, occ_map, sampling_resolution_, ivm_);
            if (occ_map) {
                doPublish(occ_map, time, map->getFrame());
                return;
            }
        }
    }
    std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
}

void OccupancyGridPublisher::publishOccupancyGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
{
    if (ivm_) {
        using local_map_t = cslibs_gridmaps::dynamic_maps::ProbabilityGridmap;
        const local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyGridMap2D>().get();
        if (m) {
            cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr occ_map(
                        new cslibs_gridmaps::static_maps::ProbabilityGridmap(
                            m->getOrigin(), m->getResolution(), m->getHeight(), m->getWidth(), ivm_->getLogOddsPrior()));

            const std::size_t chunk_step = m->getChunkSize();
            const local_map_t::index_t min_chunk_index = m->getMinChunkIndex();
            const local_map_t::index_t max_chunk_index = m->getMaxChunkIndex();
            for (int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
                for (int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {

                    local_map_t::chunk_t *chunk = m->getChunk({{j,i}});
                    if (chunk != nullptr) {
                        const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                        const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));

                        for (std::size_t k = 0 ; k < chunk_step ; ++k)
                            for (std::size_t l = 0 ; l < chunk_step ; ++l)
                                occ_map->at(cx + l, cy + k) = chunk->at(l,k);
                    }
                }
            }

            if (occ_map) {
                cslibs_gridmaps::static_maps::conversion::LogOdds::from(occ_map, occ_map);
                doPublish(occ_map, time, map->getFrame());
                return;
            }
        }
    }
    std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
}

void OccupancyGridPublisher::publishMinHeightMap2D(const map_t::ConstPtr &map,
                                                   const ros::Time &time)
{
    using local_map_t = cslibs_gridmaps::dynamic_maps::MinHeightmap;
    const local_map_t::Ptr m = map->as<cslibs_mapping::maps::MinHeightMap2D>().get();
    if (m) {
        cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr occ_map(
                    new cslibs_gridmaps::static_maps::ProbabilityGridmap(
                        m->getOrigin(), m->getResolution(), m->getHeight(), m->getWidth(), 0.5));

        const std::size_t chunk_step = m->getChunkSize();
        const local_map_t::index_t min_chunk_index = m->getMinChunkIndex();
        const local_map_t::index_t max_chunk_index = m->getMaxChunkIndex();

        for (int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
            for (int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {

                local_map_t::chunk_t *chunk = m->getChunk({{j,i}});
                if (chunk != nullptr) {
                    const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                    const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));

                    for (std::size_t k = 0 ; k < chunk_step ; ++k) {
                        for (std::size_t l = 0 ; l < chunk_step ; ++l) {
                            const double &val = chunk->at(l, k);
                            occ_map->at(cx + l, cy + k) = std::isnormal(val) ? val : 0.5;
                        }
                    }
                }
            }
        }

        if (occ_map) {
            doPublish(occ_map, time, map->getFrame());
            return;
        }
    }
    std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
}

void OccupancyGridPublisher::publishDistributionHeightMap2D(const map_t::ConstPtr &map, const ros::Time &time)
{
    using local_map_t = cslibs_gridmaps::dynamic_maps::DistributionHeightmap;
    const local_map_t::Ptr m = map->as<cslibs_mapping::maps::DistributionHeightMap2D>().get();
    if (m) {
        cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr occ_map(
                    new cslibs_gridmaps::static_maps::ProbabilityGridmap(
                        m->getOrigin(), m->getResolution(), m->getHeight(), m->getWidth(), 0.5));

        const std::size_t chunk_step = m->getChunkSize();
        const local_map_t::index_t min_chunk_index = m->getMinChunkIndex();
        const local_map_t::index_t max_chunk_index = m->getMaxChunkIndex();

        for (int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
            for (int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {

                local_map_t::chunk_t *chunk = m->getChunk({{j,i}});
                if (chunk != nullptr) {
                    const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                    const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));

                    for (std::size_t k = 0 ; k < chunk_step ; ++k) {
                        for (std::size_t l = 0 ; l < chunk_step ; ++l) {
                            const double &val = chunk->at(l, k).getMean();
                            occ_map->at(cx + l, cy + k) = std::isnormal(val) ? val : 0.5;
                        }
                    }
                }
            }
        }

        if (occ_map) {
            doPublish(occ_map, time, map->getFrame());
            return;
        }
    }
    std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
}

void OccupancyGridPublisher::doPublish(const cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr &occ_map,
                                       const ros::Time &time,
                                       const std::string &frame,
                                       const bool &normalize)
{
    if (occ_map) {
        if (normalize)
            cslibs_gridmaps::static_maps::algorithms::normalize<double>(*occ_map);

        nav_msgs::OccupancyGrid::Ptr msg;
        cslibs_gridmaps::static_maps::conversion::from(occ_map, msg);
        if (msg) {
            msg->header.stamp    = time;
            msg->header.frame_id = frame;

            publisher_.publish(msg);
            return;
        }
    }
    std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
}
}
}
