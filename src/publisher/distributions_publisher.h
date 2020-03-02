#ifndef CSLIBS_MAPPING_DISTRIBUTIONS_PUBLISHER_H
#define CSLIBS_MAPPING_DISTRIBUTIONS_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

#include <cslibs_mapping/maps/ndt_grid_map_3d.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.hpp>
#include <cslibs_mapping/maps/ndt_grid_map_2d.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_2d.hpp>

#include <cslibs_ndt_3d/conversion/distributions.hpp>
#include <cslibs_ndt_2d/conversion/distributions.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace cslibs_mapping {
namespace publisher {
template <typename T>
class DistributionsPublisherBase : public Publisher
{
private:
    using ivm_t = cslibs_gridmaps::utility::InverseModel<T>;

    virtual inline bool uses(const map_t::ConstPtr &map) const
    {
        return map->isType<cslibs_mapping::maps::NDTGridMap3D<T>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap2D<T>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<T>>();              ;
    }
    virtual inline void doAdvertise(ros::NodeHandle &nh, const std::string &topic)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const bool occupancy_ndt = nh.param<bool>(param_name("occupancy_ndt"), false);
        if (occupancy_ndt) {
            const T prob_prior    = static_cast<T>(nh.param<double>(param_name("prob_prior"),    0.5));
            const T prob_free     = static_cast<T>(nh.param<double>(param_name("prob_free"),     0.45));
            const T prob_occupied = static_cast<T>(nh.param<double>(param_name("prob_occupied"), 0.65));
            ivm_.reset(new ivm_t(
                           prob_prior, prob_free, prob_occupied));

            occ_threshold_ = static_cast<T>(nh.param<double>(param_name("occ_threshold"), 0.169));
        }

        publisher_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
    }

    virtual inline void doPublish(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<T>>())
            return publishNDTGridMap3D(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>())
            return publishOccupancyNDTGridMap3D(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap2D<T>>())
            return publishNDTGridMap2D(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap2D<T>>())
            return publishOccupancyNDTGridMap2D(map, time);
        std::cout << "[DistributionsPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
    }

    inline void publishNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = typename cslibs_mapping::maps::NDTGridMap3D<T>::map_t;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::NDTGridMap3D<T>>().get();
        if (m) {
            visualization_msgs::MarkerArray::Ptr markers(new visualization_msgs::MarkerArray);
            cslibs_ndt_3d::conversion::from(*m, *markers, time, map->getFrame());

            if (markers) {
                publisher_.publish(markers);
                return;
            }
        }
        std::cout << "[DistributionsPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    inline void publishOccupancyNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (ivm_) {
            using local_map_t = typename cslibs_mapping::maps::OccupancyNDTGridMap3D<T>::map_t;
            const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>().get();
            if (m) {
                visualization_msgs::MarkerArray::Ptr markers(new visualization_msgs::MarkerArray);
                cslibs_ndt_3d::conversion::from(*m, *markers, ivm_, time, map->getFrame(), cslibs_math_3d::Pose3<T>(), occ_threshold_);

                if (markers) {
                    publisher_.publish(markers);
                    return;
                }
            }
        }
        std::cout << "[DistributionsPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    inline void publishNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = typename cslibs_mapping::maps::NDTGridMap2D<T>::map_t;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::NDTGridMap2D<T>>().get();
        if (m) {
            visualization_msgs::MarkerArray::Ptr markers;
            cslibs_ndt_2d::conversion::from<T>(m, markers, time, map->getFrame());

            if (markers) {
                publisher_.publish(markers);
                return;
            }
        }
        std::cout << "[DistributionsPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    inline void publishOccupancyNDTGridMap2D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (ivm_) {
            using local_map_t = typename cslibs_mapping::maps::OccupancyNDTGridMap2D<T>::map_t;
            const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap2D<T>>().get();
            if (m) {
                visualization_msgs::MarkerArray::Ptr markers;
                cslibs_ndt_2d::conversion::from<T>(m, markers, ivm_, time, map->getFrame(),
                                                   cslibs_math_2d::Pose2<T>(),
                                                   cslibs_math::color::Color<T>(0.0, 0.45, 0.63),
                                                   occ_threshold_);

                if (markers) {
                    publisher_.publish(markers);
                    return;
                }
            }
        }
        std::cout << "[DistributionsPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    T                   occ_threshold_;
    typename ivm_t::Ptr ivm_;
};

using DistributionsPublisher   = DistributionsPublisherBase<double>;
using DistributionsPublisher_d = DistributionsPublisherBase<double>;
using DistributionsPublisher_f = DistributionsPublisherBase<float>;
}
}

#endif // CSLIBS_MAPPING_DISTRIBUTIONS_PUBLISHER_H
