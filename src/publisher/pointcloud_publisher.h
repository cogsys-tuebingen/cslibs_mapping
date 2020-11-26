#ifndef CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H
#define CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H

#include <cslibs_mapping/publisher/publisher.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

#include <cslibs_mapping/maps/ndt_grid_map_3d.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.hpp>

#include <cslibs_ndt/backend/octree.hpp>

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
        return map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>() ||
               map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cslibs_ndt::backend::OcTree>>() ||

               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>() ||
               map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cslibs_ndt::backend::OcTree>>();
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
        allocate_all_ = nh.param<bool>(param_name("allocate_all"), false);
        publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
    }

    virtual inline void doPublish(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>())
            return publishNDTGridMap3D<cslibs_ndt::map::tags::static_map,cis::backend::array::Array>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>())
            return publishNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::kdtree::KDTree>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>())
            return publishNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::Map>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>())
            return publishNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedMap>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>())
            return publishNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedComponentMap>(map, time);
        if (map->isType<cslibs_mapping::maps::NDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cslibs_ndt::backend::OcTree>>())
            return publishNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cslibs_ndt::backend::OcTree>(map, time);

        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::static_map,T,cis::backend::array::Array>>())
            return publishOccupancyNDTGridMap3D<cslibs_ndt::map::tags::static_map,cis::backend::array::Array>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::kdtree::KDTree>>())
            return publishOccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::kdtree::KDTree>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::Map>>())
            return publishOccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::Map>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedMap>>())
            return publishOccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedMap>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cis::backend::simple::UnorderedComponentMap>>())
            return publishOccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cis::backend::simple::UnorderedComponentMap>(map, time);
        if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,T,cslibs_ndt::backend::OcTree>>())
            return publishOccupancyNDTGridMap3D<cslibs_ndt::map::tags::dynamic_map,cslibs_ndt::backend::OcTree>(map, time);

        std::cout << "[PointcloudPublisher '" << name_ << "']: Got wrong map type!" << std::endl;
    }

    template <cslibs_ndt::map::tags::option option_t,
              template <typename, typename, typename...> class backend_t>
    inline void publishNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        using local_map_t = typename cslibs_mapping::maps::NDTGridMap3D<option_t,T,backend_t>::map_t;
        const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::NDTGridMap3D<option_t,T,backend_t>>().get();
        if (m) {
            sensor_msgs::PointCloud2 msg;
            cslibs_ndt_3d::conversion::from(*m, msg, cslibs_math_3d::Pose3<T>(), allocate_all_);

            msg.header.stamp    = time;
            msg.header.frame_id = map->getFrame();

            publisher_.publish(msg);
            return;
        }
        std::cout << "[PointcloudPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    template <cslibs_ndt::map::tags::option option_t,
              template <typename, typename, typename...> class backend_t>
    inline void publishOccupancyNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
    {
        if (ivm_) {
            using local_map_t = typename cslibs_mapping::maps::OccupancyNDTGridMap3D<option_t,T,backend_t>::map_t;
            const typename local_map_t::Ptr m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap3D<option_t,T,backend_t>>().get();
            if (m) {
                sensor_msgs::PointCloud2 msg;
                cslibs_ndt_3d::conversion::from(*m, msg, ivm_, cslibs_math_3d::Pose3<T>(), occ_threshold_, allocate_all_);

                msg.header.stamp    = time;
                msg.header.frame_id = map->getFrame();

                publisher_.publish(msg);
                return;
            }
        }
        std::cout << "[PointcloudPublisher '" << name_ << "']: Map could not be published!" << std::endl;
    }

    typename ivm_t::Ptr ivm_;
    T    occ_threshold_;
    bool allocate_all_;
};

using PointcloudPublisher   = PointcloudPublisherBase<double>;
using PointcloudPublisher_d = PointcloudPublisherBase<double>;
using PointcloudPublisher_f = PointcloudPublisherBase<float>;
}
}

#endif // CSLIBS_MAPPING_POINTCLOUD_PUBLISHER_H
