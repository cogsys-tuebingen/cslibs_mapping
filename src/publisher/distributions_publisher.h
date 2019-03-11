#ifndef CSLIBS_MAPPING_DISTRIBUTIONS_PUBLISHER_H
#define CSLIBS_MAPPING_DISTRIBUTIONS_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

#include <cslibs_mapping/maps/ndt_grid_map_3d.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.hpp>

#include <cslibs_ndt_3d/conversion/distributions.hpp>

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
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>();
    }
    virtual inline void doAdvertise(ros::NodeHandle &nh, const std::string &topic)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const bool occupancy_ndt = nh.param<bool>(param_name("occupancy_ndt"), false);
        if (occupancy_ndt) {
            const double prob_prior    = nh.param(param_name("prob_prior"),    0.5);
            const double prob_free     = nh.param(param_name("prob_free"),     0.45);
            const double prob_occupied = nh.param(param_name("prob_occupied"), 0.65);
            ivm_.reset(new ivm_t(
                           prob_prior, prob_free, prob_occupied));

            occ_threshold_ = static_cast<T>(nh.param<double>(param_name("occ_threshold"), 0.169));
        }

        publisher_ = nh.advertise<cslibs_ndt_3d::DistributionArray>(topic, 1);
    }

    virtual inline void doPublish(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<T>>())
            return publishNDTGridMap3D(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>())
            return publishOccupancyNDTGridMap3D(map, time);
        std::cout << "[DistributionsPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
    }

    inline void publishNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = cslibs_ndt_3d::dynamic_maps::Gridmap<T>;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::NDTGridMap3D<T>>().get();
        if (m) {
            cslibs_ndt_3d::DistributionArray::Ptr distributions;
            cslibs_ndt_3d::conversion::from<T>(m, distributions);

            if (distributions) {
                distributions->header.stamp    = time;
                distributions->header.frame_id = map->getFrame();

                publisher_.publish(distributions);
                return;
            }
        }
        std::cout << "[DistributionsPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    inline void publishOccupancyNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (ivm_) {
            using local_map_t = cslibs_ndt_3d::dynamic_maps::OccupancyGridmap<T>;
            const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>().get();
            if (m) {
                cslibs_ndt_3d::DistributionArray::Ptr distributions;
                cslibs_ndt_3d::conversion::from<T>(m, distributions, ivm_, occ_threshold_);

                if (distributions) {
                    distributions->header.stamp    = time;
                    distributions->header.frame_id = map->getFrame();

                    publisher_.publish(distributions);
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
