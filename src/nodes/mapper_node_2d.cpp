#include "mapper_node_2d.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <cslibs_math_2d/linear/polar_pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

namespace cslibs_mapping {
MapperNode2d::MapperNode2d() :
    nh_("~")
{
}

bool MapperNode2d::setup()
{
    ROS_INFO_STREAM("Setting up subscribers");
    const int           subscriber_queue_size       = nh_.param<int>("subscriber_queue_size", 1);
    const double        occ_grid_resolution         = nh_.param<double>("occ_grid_resolution", 0.05);
    const double        occ_grid_chunk_resolution   = nh_.param<double>("occ_chunk_resolution", 5.0);
    const std::string   occ_map_topic               = nh_.param<std::string>("occ_map_topic", "/map/occ");
    const double        occ_map_pub_rate            = nh_.param<double>("occ_map_pub_rate",  10.0);
    const double        occ_map_prob_free           = nh_.param<double>("occ_map_prob_free", 0.45);
    const double        occ_map_prob_occ            = nh_.param<double>("occ_map_prob_occ",  0.55);
    const double        occ_map_prob_prior          = nh_.param<double>("occ_map_prob_prior", 0.5);

    const std::string   ndt_map_topic               = nh_.param<std::string>("ndt_map_topic", "/map/ndt");
    const double        ndt_map_pub_rate            = nh_.param<double>("ndt_map_pub_rate", 10.0);
    const double        ndt_grid_resolution         = nh_.param<double>("ndt_grid_resolution", 1.0);
    const double        ndt_sampling_resolution     = nh_.param<double>("ndt_sampling_resolution", 0.05);

    std::vector<std::string> lasers;
    if(!nh_.getParam("lasers", lasers)) {
        ROS_ERROR_STREAM("Did not find any laser inputs!");
        return false;
    }

    map_frame_                = nh_.param<std::string>("map_frame", "/odom");

    node_rate_                = nh_.param<double>("rate", 0.0);
    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");

    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");
    tf_timeout_               = ros::Duration(nh_.param("tf_timeout", 0.1));

    linear_interval_[0]       = nh_.param<double>("range_min", 0.05);
    linear_interval_[1]       = nh_.param<double>("range_max", 30.0);
    angular_interval_[0]      = nh_.param<double>("angle_min",-M_PI);
    angular_interval_[1]      = nh_.param<double>("angle_max", M_PI);


    cslibs_gridmaps::utility::InverseModel inverse_model(occ_map_prob_prior, occ_map_prob_free, occ_map_prob_occ);
    occ_mapper_.reset(new OccupancyGridMapper2d(inverse_model,
                                                occ_grid_resolution,
                                                occ_grid_chunk_resolution,
                                                map_frame_));
    occ_mapper_->setCallback(OccupancyGridMapper2d::callback_t::from<MapperNode2d, &MapperNode2d::publishOcc>(this));

    ndt_mapper_.reset(new NDTGridMapper2d(ndt_grid_resolution, ndt_sampling_resolution, map_frame_));
    ndt_mapper_->setCallback(OccupancyGridMapper2d::callback_t::from<MapperNode2d, &MapperNode2d::publishNDT>(this));

    for(const auto &l : lasers) {
        ROS_INFO_STREAM("Subscribing to laser '" << l << "'");
        sub_lasers_.emplace_back(nh_.subscribe(l,
                                               static_cast<unsigned int>(subscriber_queue_size),
                                               &MapperNode2d::laserscan,
                                               this));
    }
    pub_occ_map_            = nh_.advertise<nav_msgs::OccupancyGrid>(occ_map_topic, 1);
    pub_occ_interval_       = ros::Duration(occ_map_pub_rate > 0.0 ? 1.0 / occ_map_pub_rate : 0.0);
    pub_occ_last_time_      = ros::Time::now();

    pub_ndt_map_            = nh_.advertise<nav_msgs::OccupancyGrid>(ndt_map_topic, 1);
    pub_ndt_interval_       = ros::Duration(occ_map_pub_rate > 0.0 ? 1.0 / ndt_map_pub_rate : 0.0);
    pub_ndt_last_time_      = ros::Time::now();

    tf_.reset(new cslibs_math_ros::tf::TFListener2d);

    ROS_INFO_STREAM("Setup succesful!");
    return true;
}

void MapperNode2d::run()
{
    ros::Rate r(node_rate_);
    while(ros::ok()) {
        const ros::Time now = ros::Time::now();
        if(pub_occ_last_time_.isZero())
            pub_occ_last_time_ = now;

        if(pub_occ_interval_.isZero() || pub_occ_last_time_ + pub_occ_interval_ < now) {
            occ_mapper_->requestMap();
        }
        if(pub_ndt_interval_.isZero() || (pub_ndt_last_time_ + pub_ndt_interval_ < now)) {
            ndt_mapper_->requestMap();
        }
        r.sleep();
        ros::spinOnce();
    }
}

void MapperNode2d::laserscan(const sensor_msgs::LaserScanConstPtr &msg)
{
    cslibs_math_2d::PolarPointlcoud2d::Ptr laserscan;
    if(undistortion_) {
        cslibs_math_ros::sensor_msgs::conversion_2d::from(msg,
                                                          linear_interval_, angular_interval_,
                                                          undistortion_fixed_frame_, tf_timeout_, *tf_,
                                                          laserscan);
    }
    if(!laserscan) {
        cslibs_math_ros::sensor_msgs::conversion_2d::from(msg, linear_interval_, angular_interval_, laserscan);
    }
    if(!laserscan)
        return;

    cslibs_math_2d::Transform2d o_T_l;
    cslibs_time::TimeFrame time_frame = cslibs_math_ros::sensor_msgs::conversion_2d::from(msg);
    if(tf_->lookupTransform(map_frame_, msg->header.frame_id,
                            ros::Time(time_frame.end.seconds()),
                            o_T_l,
                            tf_timeout_)) {

        cslibs_math_2d::Pointcloud2d::Ptr points(new cslibs_math_2d::Pointcloud2d);
        measurement_t  m(points, o_T_l, time_frame.end);
        for(auto it = laserscan->begin() ; it != laserscan->end() ; ++it) {
            if(it->isNormal())
                points->insert(it->getCartesian());
        }
        occ_mapper_->insert(m);
        ndt_mapper_->insert(m);
    }
}

void MapperNode2d::publishNDT(const OccupancyGridMapper2d::static_map_stamped_t &map)
{
    if(map.data()) {
        nav_msgs::OccupancyGrid::Ptr msg;
        cslibs_gridmaps::static_maps::conversion::from(map.data(), msg);
        msg->header.stamp.fromNSec(static_cast<uint64_t>(map.stamp().nanoseconds()));
        msg->header.frame_id = map_frame_;
        pub_ndt_map_.publish(msg);
        pub_ndt_last_time_ = ros::Time::now();
    }
}

void MapperNode2d::publishOcc(const OccupancyGridMapper2d::static_map_stamped_t &map)
{
    if(map.data()) {
        nav_msgs::OccupancyGrid::Ptr msg;
        cslibs_gridmaps::static_maps::conversion::from(map.data(), msg);
        msg->header.stamp.fromNSec(static_cast<uint64_t>(map.stamp().nanoseconds()));
        msg->header.frame_id = map_frame_;
        pub_occ_map_.publish(msg);
        pub_occ_last_time_ = ros::Time::now();
    }
}

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl_2d_mapping_ocm_node");
    cslibs_mapping::MapperNode2d instance;
    instance.setup();
    instance.run();

    return 0;
}
