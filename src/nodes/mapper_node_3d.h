#ifndef MAPPER_NODE_3D_H
#define MAPPER_NODE_3D_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cslibs_mapping/mapper/ndt_grid_mapper_2d.h>
#include <cslibs_mapping/mapper/ndt_grid_mapper_3d.h>
#include <cslibs_mapping/mapper/occupancy_grid_mapper_2d.h>
#include <cslibs_mapping/mapper/occupancy_ndt_grid_mapper_2d.h>
#include <cslibs_mapping/mapper/occupancy_ndt_grid_mapper_3d.h>
#include <cslibs_mapping/mapper/octomap_mapper_3d.h>
#include <cslibs_mapping/SaveMap.h>

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_math_ros/tf/tf_listener_2d.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>
#include <cslibs_math_2d/linear/polar_pointcloud.hpp>

#include <ndt_map/ndt_map.h>

namespace cslibs_mapping {
class MapperNode3d
{    
private:
    using interval_t        = std::array<float, 2>;
    using occ_map_2d_t      = OccupancyGridMapper2d;
    using ndt_map_2d_t      = NDTGridMapper2d;
    using occ_ndt_map_2d_t  = OccupancyNDTGridMapper2d;
    using occ_map_3d_t      = OctomapMapper3d;
    using ndt_map_3d_t      = NDTGridMapper3d;
    using occ_ndt_map_3d_t  = OccupancyNDTGridMapper3d;
    using msg_2d_t          = nav_msgs::OccupancyGrid;
    using msg_3d_t          = sensor_msgs::PointCloud2;
    using octomap_msg_3d_t  = octomap_msgs::Octomap;
    using point_2d_t        = cslibs_math_2d::Point2d;
    using transform_2d_t    = cslibs_math_2d::Transform2d;
    using measurement_2d_t  = Measurement<point_2d_t, transform_2d_t>;
    using point_3d_t        = cslibs_math_3d::Point3d;
    using transform_3d_t    = cslibs_math_3d::Transform3d;
    using measurement_3d_t  = Measurement<point_3d_t, transform_3d_t>;

    template <typename map_t, typename msg_t>
    struct MapperWorker {
        typename map_t::Ptr mapper_;
        std::string         map_frame_;
        ros::Publisher      pub_map_;
        ros::Duration       pub_interval_;
        ros::Time           pub_last_time_;

        ros::Publisher      pub_marker_;
        ros::Publisher      pub_distributions_;

        bool                active_;

        template <typename t = msg_t>
        typename std::enable_if<!std::is_same<t, msg_3d_t>::value, void>::type
        setCallback()
        {
            mapper_->setCallback(
                        map_t::callback_t::template from<MapperWorker<map_t, msg_t>,
                        &MapperWorker<map_t, msg_t>::publish>(this));
        }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_3d_t>::value, void>::type
        setCallback()
        {
            mapper_->setCallback(
                        map_t::callback_t::template from<MapperWorker<map_t, msg_t>,
                        &MapperWorker<map_t, msg_t>::publish>(this));

            mapper_->setMarkerCallback(
                        map_t::marker_callback_t::template from<MapperWorker<map_t, msg_t>,
                        &MapperWorker<map_t, msg_t>::publish>(this));

            mapper_->setDistributionsCallback(
                        map_t::distributions_callback_t::template from<MapperWorker<map_t, msg_t>,
                        &MapperWorker<map_t, msg_t>::publish>(this));
        }

        void setup(
                ros::NodeHandle   & nh,
                const std::string & topic,
                const double      & rate,
                const ros::Time   & now,
                const bool        & active)
        {
            pub_map_       = nh.advertise<msg_t>(topic, 1);
            pub_interval_  = ros::Duration(rate > 0.0 ? 1.0 / rate : 0.0);
            pub_last_time_ = now;            

            setupAdditionalPublisher(nh, topic);
            active_ = active;
        }

        void requestMap(
                const ros::Time & now)
        {
            if(pub_last_time_.isZero())
                pub_last_time_ = now;

            if (pub_interval_.isZero() || (pub_last_time_ + pub_interval_ < now))
                mapper_->requestMap();
        }

        template <typename t = msg_t>
        typename std::enable_if<!std::is_same<t, msg_3d_t>::value, void>::type
        setupAdditionalPublisher(
                ros::NodeHandle & nh,
                const std::string & topic)
        { }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_3d_t>::value, void>::type
        setupAdditionalPublisher(
                ros::NodeHandle   & nh,
                const std::string & topic)
        {
            pub_marker_        = nh.advertise<visualization_msgs::MarkerArray>(topic + "/marker", 1);
            pub_distributions_ = nh.advertise<Distribution3dArray>(topic + "/distributions", 1);
        }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_2d_t>::value, void>::type
        publish(
                const typename map_t::static_map_stamped_t & map)
        {
            if(map.data()) {
                typename msg_t::Ptr msg;
                cslibs_gridmaps::static_maps::conversion::from(map.data(), msg);
                msg->header.stamp.fromNSec(static_cast<uint64_t>(map.stamp().nanoseconds()));
                msg->header.frame_id = map_frame_;
                pub_map_.publish(msg);
                pub_last_time_ = ros::Time::now();
            }
        }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_3d_t>::value, void>::type
        publish(
                const typename map_t::static_map_stamped_t & map)
        {
            if (map.data()) {
                typename msg_t::Ptr msg(new msg_t());
                pcl::toROSMsg(*(map.data()), *msg);
                msg->header.stamp.fromNSec(static_cast<uint64_t>(map.stamp().nanoseconds()));
                msg->header.frame_id = map_frame_;
                pub_map_.publish(msg);
                pub_last_time_ = ros::Time::now();
            }
        }        

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_3d_t>::value, void>::type
        publish(
                const visualization_msgs::MarkerArrayPtr & msg)
        {
            if (msg) {
                for (auto & marker : msg->markers) {
                    marker.header.stamp    = pub_last_time_;
                    marker.header.frame_id = map_frame_;
                }
                pub_marker_.publish(msg);
            }
        }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_3d_t>::value, void>::type
        publish(
                const Distribution3dArray::Ptr & msg)
        {
            if (msg)
                pub_distributions_.publish(msg);
        }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, octomap_msg_3d_t>::value, void>::type
        publish(
                const typename map_t::static_map_stamped_t & map)
        {
            if(map.data())
                pub_map_.publish(*(map.data()));
        }
    };

public:
    MapperNode3d();
    ~MapperNode3d();
    bool setup();
    void run();

private:
    ros::NodeHandle                          nh_;
    std::vector<ros::Subscriber>             sub_lasers_;

    cslibs_math_ros::tf::TFListener2d::Ptr   tf_;
    double                                   node_rate_;

    // possible maps
    MapperWorker<occ_map_2d_t, msg_2d_t>         occ_2d_mapper_;
    MapperWorker<ndt_map_2d_t, msg_2d_t>         ndt_2d_mapper_;
    MapperWorker<occ_ndt_map_2d_t, msg_2d_t>     occ_ndt_2d_mapper_;
    MapperWorker<occ_map_3d_t, octomap_msg_3d_t> occ_3d_mapper_;
    MapperWorker<ndt_map_3d_t, msg_3d_t>         ndt_3d_mapper_;
    MapperWorker<occ_ndt_map_3d_t, msg_3d_t>     occ_ndt_3d_mapper_;

    // Örebro NDT-OM map
    bool                                         ndt_3d_map_oru_active_;
    bool                                         ndt_3d_map_oru_pub_active_;
    std::shared_ptr<lslgeneric::NDTMap>          ndt_3d_map_oru_;
    ros::Publisher                               ndt_3d_map_oru_pub_;
    cslibs_math::statistics::Distribution<1, 3>  ndt_3d_map_oru_stats_;

    // Örebro NDT-OM map
    bool                                         ndt_2d_map_oru_active_;
    bool                                         ndt_2d_map_oru_pub_active_;
    std::shared_ptr<lslgeneric::NDTMap>          ndt_2d_map_oru_;
    ros::Publisher                               ndt_2d_map_oru_pub_;
    cslibs_math::statistics::Distribution<1, 3>  ndt_2d_map_oru_stats_;

    bool                                     undistortion_;              /// check if undistortion shall be applied
    std::string                              undistortion_fixed_frame_;  /// the fixed frame necessary for the undistortion
    ros::Duration                            tf_timeout_;                /// time out for the tf listener

    interval_t                               linear_interval_;           /// linear field of view
    interval_t                               angular_interval_;          /// angular field of view

    // map saving
    ros::ServiceServer                       service_save_map_;
    std::string                              output_path_;

    // path
    std::string                              base_frame_;
    std::string                                  map_frame_;
    nav_msgs::Path                           path_;
    ros::Publisher                           pub_path_;
    ros::Duration                            path_update_interval_;

    bool                                     filter_laserscan3d_;
    double                                   filter_size_;

    // 2d and 3d laser callbacks
    void laserscan2d(
            const sensor_msgs::LaserScanConstPtr & msg);

    void laserscan3d(
            const sensor_msgs::PointCloud2ConstPtr & msg);

    template <typename map_t>
    void insert(
            MapperWorker<map_t, msg_2d_t>                & mapper,
            const std::string                            & frame,
            const ros::Time                              & time,
            const cslibs_math_2d::PolarPointlcoud2d::Ptr & laserscan)
    {
        cslibs_math_2d::Transform2d o_T_l;
        if(tf_->lookupTransform(mapper.map_frame_,
                                frame,
                                time,
                                o_T_l,
                                tf_timeout_)) {

            cslibs_math_2d::Pointcloud2d::Ptr points(new cslibs_math_2d::Pointcloud2d);
            measurement_2d_t m(points, o_T_l, cslibs_time::Time(time.toNSec()));
            for(auto it = laserscan->begin() ; it != laserscan->end() ; ++ it)
                if(it->isNormal())
                    points->insert(it->getCartesian());

            if (mapper.active_)
                mapper.mapper_->insert(m);
            updatePath(time);
        }
    }

    template <typename map_t, typename msg_t, typename point_t>
    typename std::enable_if<!std::is_same<msg_t, msg_2d_t>::value, void>::type
    insert(
            MapperWorker<map_t, msg_t>                        & mapper,
            const std::string                                 & frame,
            const ros::Time                                   & time,
            const typename pcl::PointCloud<point_t>::ConstPtr & laserscan)
    {
        tf::Transform o_T_l;
        uint64_t nanoseconds = laserscan->header.stamp * 1e3;
        if (tf_->lookupTransform(mapper.map_frame_,
                                 frame,
                                 time,
                                 o_T_l,
                                 tf_timeout_)) {

            transform_3d_t origin = cslibs_math_ros::tf::conversion_3d::from(o_T_l);
            cslibs_math_3d::Pointcloud3d::Ptr points(new cslibs_math_3d::Pointcloud3d);
            measurement_3d_t m(points,
                               origin,
                               cslibs_time::Time(nanoseconds));

            for(auto it = laserscan->begin() ; it != laserscan->end() ; ++ it) {
                cslibs_math_3d::Point3d::arr_t arr =  {static_cast<double>(it->x),
                                                       static_cast<double>(it->y),
                                                       static_cast<double>(it->z)};
                if (std::isnormal(arr[0]) && std::isnormal(arr[1]) && std::isnormal(arr[2]) && arr[2] < 10000)
                    points->insert(cslibs_math_3d::Point3d(arr));
            }
            if (mapper.active_)
                mapper.mapper_->insert(m);
            updatePath(time);
        }
    }

    // map saving
    bool saveMap(
        SaveMap::Request                  & req,
        cslibs_mapping::SaveMap::Response &);

    bool saveMap(
        const std::string & path);

    void updatePath(
        const ros::Time & time);
};
}

#endif // MAPPER_NODE_3D_H
