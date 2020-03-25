#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_PUBLISHER_H
#define CSLIBS_MAPPING_OCCUPANCY_GRID_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>

#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

#include <cslibs_mapping/maps/ndt_grid_map_2d.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_2d.hpp>
#include <cslibs_mapping/maps/occupancy_grid_map_2d.hpp>
#include <cslibs_mapping/maps/min_height_map_2d.hpp>
#include <cslibs_mapping/maps/distribution_height_map_2d.hpp>

#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>
#include <cslibs_ndt_2d/conversion/merge.hpp>

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>

#include <nav_msgs/OccupancyGrid.h>

namespace cslibs_mapping {
namespace publisher {
template <typename Tp, typename T>
class OccupancyGridPublisherBase : public Publisher
{
private:
    using ivm_t = cslibs_gridmaps::utility::InverseModel<T>;

    virtual inline bool uses(const map_t::ConstPtr &map) const
    {
        return map->isType<cslibs_mapping::maps::OccupancyGridMap2D<Tp,T>>() ||

               map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>() ||

               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>() ||

               map->isType<cslibs_mapping::maps::MinHeightMap2D<Tp,T>>() ||
               map->isType<cslibs_mapping::maps::DistributionHeightMap2D<Tp,T>>();
    }

    virtual inline void doAdvertise(ros::NodeHandle &nh, const std::string &topic)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        sampling_resolution_ = static_cast<T>(nh.param<double>(param_name("sampling_resolution"), 0.025));
        default_value_       = static_cast<T>(nh.param<double>(param_name("default_value"), 0.0));
        const bool occupancy = nh.param<bool>(param_name("occupancy"), false);
        if (occupancy) {
            const T prob_prior    = static_cast<T>(nh.param<double>(param_name("prob_prior"),    0.5));
            const T prob_free     = static_cast<T>(nh.param<double>(param_name("prob_free"),     0.45));
            const T prob_occupied = static_cast<T>(nh.param<double>(param_name("prob_occupied"), 0.65));
            ivm_.reset(new ivm_t(
                           prob_prior, prob_free, prob_occupied));
        }
        flattened_ = nh.param<bool>(param_name("flatten"), false);
        if(flattened_) {
          ROS_WARN_STREAM("Publishing the flatttened map.");
        }
        interpolate_  = nh.param<bool>(param_name("interpolate"),false);
        allocate_all_ = nh.param<bool>(param_name("allocate_all"),false);

        publisher_ = nh.advertise<nav_msgs::OccupancyGrid>(topic, 1);
    }

    virtual inline void doPublish(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (map->isType<cslibs_mapping::maps::OccupancyGridMap2D<Tp,T>>())
            return publishOccupancyGridMap2D(map, time);

        if (map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>())
            return publishNDTGridMap2D<cslibs_ndt::map::tags::static_map,cis::backend::array::Array>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>())
            return publishNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::kdtree::KDTree>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>())
            return publishNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::Map>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>())
            return publishNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedMap>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>())
            return publishNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedComponentMap>(map, time);

        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>())
            return publishOccupancyNDTGridMap2D<cslibs_ndt::map::tags::static_map,cis::backend::array::Array>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>())
            return publishOccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::kdtree::KDTree>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>())
            return publishOccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::Map>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>())
            return publishOccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedMap>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>())
            return publishOccupancyNDTGridMap2D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedComponentMap>(map, time);

        if (map->isType<cslibs_mapping::maps::MinHeightMap2D<Tp,T>>())
            return publishMinHeightMap2D(map, time);
        if (map->isType<cslibs_mapping::maps::DistributionHeightMap2D<Tp,T>>())
            return publishDistributionHeightMap2D(map, time);
        std::cout << "[OccupancyGridPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
    }

    inline void publishOccupancyGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (ivm_) {
            using local_map_t = typename cslibs_mapping::maps::OccupancyGridMap2D<Tp,T>::map_t;//cslibs_gridmaps::dynamic_maps::ProbabilityGridmap<Tp,T>;
            const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyGridMap2D<Tp,T>>().get();
            if (m) {
                typename cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>::Ptr occ_map(
                            new cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>(
                                m->getOrigin(), m->getResolution(), m->getHeight(), m->getWidth(), ivm_->getLogOddsPrior()));

                const std::size_t chunk_step = m->getChunkSize();
                const typename local_map_t::index_t min_chunk_index = m->getMinChunkIndex();
                const typename local_map_t::index_t max_chunk_index = m->getMaxChunkIndex();
                for (int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
                    for (int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {

                        typename local_map_t::chunk_t *chunk = m->getChunk({{j,i}});
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
                    cslibs_gridmaps::static_maps::conversion::LogOdds::from<Tp,T>(occ_map, occ_map);
                    doPublish(occ_map, time, map->getFrame());
                    return;
                }
            }
        }
        std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    template <cslibs_ndt::map::tags::option option_t,
              template <typename, typename, typename...> class backend_t>
    inline void publishNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = typename cslibs_mapping::maps::NDTGridMap2D<option_t,T,backend_t>::map_t;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::NDTGridMap2D<option_t,T,backend_t>>().get();
        if (m && !m->empty()) {
            typename cslibs_gridmaps::static_maps::ProbabilityGridmap<T,T>::Ptr occ_map;

            //if(flattened_) {
              //std::cerr << "flattening" << std::endl;
              //typename cslibs_ndt_2d::static_maps::mono::Gridmap<T>::Ptr fm = cslibs_ndt_2d::conversion::merge<T>(m);
              //std::cerr << "flattened" << std::endl;
              //cslibs_ndt_2d::conversion::from(*fm, occ_map, sampling_resolution_);
            //} else {
              cslibs_ndt_2d::conversion::from(*m, occ_map,
                                              sampling_resolution_, allocate_all_, default_value_, interpolate_);
            //}
            if (occ_map) {
                doPublish<T,T>(occ_map, time, map->getFrame());
                return;
            }
        }
        std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    template <cslibs_ndt::map::tags::option option_t,
              template <typename, typename, typename...> class backend_t>
    inline void publishOccupancyNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (ivm_) {
            using local_map_t = typename cslibs_mapping::maps::OccupancyNDTGridMap2D<option_t,T,backend_t>::map_t;
            const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap2D<option_t,T,backend_t>>().get();
            if (m && !m->empty()) {
                typename cslibs_gridmaps::static_maps::ProbabilityGridmap<T,T>::Ptr occ_map;
                cslibs_ndt_2d::conversion::from(*m, occ_map,
                                                sampling_resolution_, ivm_, allocate_all_, default_value_, interpolate_);
                if (occ_map) {
                    doPublish<T,T>(occ_map, time, map->getFrame());
                    return;
                }
            }
        }
        std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    inline void publishMinHeightMap2D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = typename cslibs_mapping::maps::MinHeightMap2D<Tp,T>::map_t;//cslibs_gridmaps::dynamic_maps::MinHeightmap<Tp,T>;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::MinHeightMap2D<Tp,T>>().get();
        if (m) {
            typename cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>::Ptr occ_map(
                        new cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>(
                            m->getOrigin(), m->getResolution(), m->getHeight(), m->getWidth(), 0.5));

            const std::size_t chunk_step = m->getChunkSize();
            const typename local_map_t::index_t min_chunk_index = m->getMinChunkIndex();
            const typename local_map_t::index_t max_chunk_index = m->getMaxChunkIndex();

            for (int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
                for (int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {

                    typename local_map_t::chunk_t *chunk = m->getChunk({{j,i}});
                    if (chunk != nullptr) {
                        const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                        const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));

                        for (std::size_t k = 0 ; k < chunk_step ; ++k) {
                            for (std::size_t l = 0 ; l < chunk_step ; ++l) {
                                const T &val = chunk->at(l, k);
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

    inline void publishDistributionHeightMap2D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = typename cslibs_mapping::maps::DistributionHeightMap2D<Tp,T>::map_t;//cslibs_gridmaps::dynamic_maps::DistributionHeightmap<Tp,T>;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::DistributionHeightMap2D<Tp,T>>().get();
        if (m) {
            typename cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>::Ptr occ_map(
                        new cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>(
                            m->getOrigin(), m->getResolution(), m->getHeight(), m->getWidth(), 0.5));

            const std::size_t chunk_step = m->getChunkSize();
            const typename local_map_t::index_t min_chunk_index = m->getMinChunkIndex();
            const typename local_map_t::index_t max_chunk_index = m->getMaxChunkIndex();

            for (int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
                for (int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {

                    typename local_map_t::chunk_t *chunk = m->getChunk({{j,i}});
                    if (chunk != nullptr) {
                        const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                        const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));

                        for (std::size_t k = 0 ; k < chunk_step ; ++k) {
                            for (std::size_t l = 0 ; l < chunk_step ; ++l) {
                                const T &val = chunk->at(l, k).getMean();
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

    template <typename TTp = Tp, typename TT = T>
    inline void doPublish(const typename cslibs_gridmaps::static_maps::ProbabilityGridmap<TTp,TT>::Ptr &occ_map,
                          const ros::Time &time, const std::string &frame, const bool &normalize = false)
    {
        if (occ_map) {
            if (normalize)
                cslibs_gridmaps::static_maps::algorithms::normalize<TTp,TT>(*occ_map);

            nav_msgs::OccupancyGrid::Ptr msg;
            cslibs_gridmaps::static_maps::conversion::from<TTp,TT>(*occ_map, msg);
            if (msg) {
                msg->header.stamp    = time;
                msg->header.frame_id = frame;

                publisher_.publish(msg);
                return;
            }
        }
        std::cout << "[OccupancyGridPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    T                   sampling_resolution_;
    T                   default_value_;
    typename ivm_t::Ptr ivm_;
    bool                flattened_;
    bool                interpolate_;
    bool                allocate_all_;
};

using OccupancyGridPublisher    = OccupancyGridPublisherBase<double,double>;
using OccupancyGridPublisher_dd = OccupancyGridPublisherBase<double,double>;
using OccupancyGridPublisher_df = OccupancyGridPublisherBase<double,float>;
using OccupancyGridPublisher_ff = OccupancyGridPublisherBase<float,float>;
using OccupancyGridPublisher_fd = OccupancyGridPublisherBase<float,double>;
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_PUBLISHER_H
