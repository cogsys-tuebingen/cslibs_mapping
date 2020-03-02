#ifndef CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H
#define CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

#include <cslibs_mapping/maps/ndt_grid_map_3d.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.hpp>

#include <cslibs_ndt_3d/conversion/sensor_msgs_pointcloud2.hpp>
#include <cslibs_ndt_3d/conversion/sensor_msgs_pointcloud2_rgb.hpp>

namespace cslibs_mapping {
namespace publisher {
template <typename T>
class PointcloudPublisherBase : public Publisher
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
            const T prob_prior    = static_cast<T>(nh.param<double>(param_name("prob_prior"),    0.5));
            const T prob_free     = static_cast<T>(nh.param<double>(param_name("prob_free"),     0.45));
            const T prob_occupied = static_cast<T>(nh.param<double>(param_name("prob_occupied"), 0.65));
            ivm_.reset(new ivm_t(
                           prob_prior, prob_free, prob_occupied));

            occ_threshold_ = static_cast<T>(nh.param<double>(param_name("occ_threshold"), 0.169));
        }
        publish_sampled_ = nh.param<bool>(param_name("sample"), true);
        allocate_all_ = nh.param<bool>(param_name("allocate_all"), false);
        sampling_resolution_ = static_cast<T>(nh.param<double>(param_name("sampling_resolution"), 0.1));

        publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
    }

    virtual inline void doPublish(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<T>>())
            return publishNDTGridMap3D(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>())
            return publishOccupancyNDTGridMap3D(map, time);
        std::cout << "[PointcloudPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
    }

    inline void publishNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = cslibs_ndt_3d::dynamic_maps::Gridmap<T>;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::NDTGridMap3D<T>>().get();
        if (m) {
            sensor_msgs::PointCloud2 msg;

            if (publish_sampled_)
                cslibs_ndt_3d::conversion::/*rgbFrom*/from<T>(m, msg, /*sampling_resolution_,*/ occ_threshold_, allocate_all_);
            else
                cslibs_ndt_3d::conversion::from<T>(m, msg);

            msg.header.stamp    = time;
            msg.header.frame_id = map->getFrame();

            publisher_.publish(msg);
            return;
        }
        std::cout << "[PointcloudPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    inline void publishOccupancyNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (ivm_) {
            using local_map_t = cslibs_ndt_3d::dynamic_maps::OccupancyGridmap<T>;
            const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap3D<T>>().get();
            if (m) {
                sensor_msgs::PointCloud2 msg;

                if (publish_sampled_)
                    cslibs_ndt_3d::conversion::/*rgbFrom*/from<T>(m, msg, ivm_, cslibs_math_3d::Pose3<T>(),/*sampling_resolution_,*/ occ_threshold_, allocate_all_);
                else
                    cslibs_ndt_3d::conversion::from<T>(m, msg, ivm_, cslibs_math_3d::Pose3<T>(), occ_threshold_);

                msg.header.stamp    = time;
                msg.header.frame_id = map->getFrame();

                publisher_.publish(msg);
                return;
            }
        }
        std::cout << "[PointcloudPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    T                   occ_threshold_;
    typename ivm_t::Ptr ivm_;

    bool publish_sampled_;
    bool allocate_all_;
    T    sampling_resolution_;
};

using PointcloudPublisher   = PointcloudPublisherBase<double>;
using PointcloudPublisher_d = PointcloudPublisherBase<double>;
using PointcloudPublisher_f = PointcloudPublisherBase<float>;
}
}

#endif // CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H
