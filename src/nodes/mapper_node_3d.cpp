#include "mapper_node_3d.h"

#include <pcl/filters/filter.h>
#include <cslibs_time/time.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>

namespace cslibs_mapping {
MapperNode3d::MapperNode3d() :
    nh_("~")
{ }

bool MapperNode3d::setup()
{
    ROS_INFO_STREAM("Setting up subscribers");
    const int           subscriber_queue_size          = nh_.param<int>("subscriber_queue_size", 1);
    const double        occ_map_prob_prior             = nh_.param<double>("occ_map_prob_prior", 0.5);
    const double        occ_map_prob_free              = nh_.param<double>("occ_map_prob_free", 0.45);
    const double        occ_map_prob_occ               = nh_.param<double>("occ_map_prob_occ", 0.55);

    const double        occ_2d_grid_resolution         = nh_.param<double>("occ_2d_grid_resolution", 0.05);
    const double        occ_2d_grid_chunk_resolution   = nh_.param<double>("occ_2d_chunk_resolution", 5.0);
    const std::string   occ_2d_map_topic               = nh_.param<std::string>("occ_2d_map_topic", "/map/2d/occ");
    const double        occ_2d_map_pub_rate            = nh_.param<double>("occ_2d_map_pub_rate", 10.0);

    const double        ndt_2d_grid_resolution         = nh_.param<double>("ndt_2d_grid_resolution", 1.0);
    const double        ndt_2d_sampling_resolution     = nh_.param<double>("ndt_2d_sampling_resolution", 0.025);
    const std::string   ndt_2d_map_topic               = nh_.param<std::string>("ndt_2d_map_topic", "/map/2d/ndt");
    const double        ndt_2d_map_pub_rate            = nh_.param<double>("ndt_2d_map_pub_rate", 10.0);

    const double        occ_ndt_2d_grid_resolution     = nh_.param<double>("occ_ndt_2d_grid_resolution", 1.0);
    const double        occ_ndt_2d_sampling_resolution = nh_.param<double>("occ_ndt_2d_sampling_resolution", 0.025);
    const std::string   occ_ndt_2d_map_topic           = nh_.param<std::string>("occ_ndt_2d_map_topic", "/map/2d/occ_ndt");
    const double        occ_ndt_2d_map_pub_rate        = nh_.param<double>("occ_ndt_2d_map_pub_rate", 10.0);

    const double        ndt_3d_grid_resolution         = nh_.param<double>("ndt_3d_grid_resolution", 2.0);
    const double        ndt_3d_sampling_resolution     = nh_.param<double>("ndt_3d_sampling_resolution", 0.25);
    const std::string   ndt_3d_map_topic               = nh_.param<std::string>("ndt_3d_map_topic", "/map/3d/ndt");
    const double        ndt_3d_map_pub_rate            = nh_.param<double>("ndt_3d_map_pub_rate", 10.0);

    const double        occ_ndt_3d_grid_resolution     = nh_.param<double>("occ_ndt_3d_grid_resolution", 2.0);
    const double        occ_ndt_3d_sampling_resolution = nh_.param<double>("occ_ndt_3d_sampling_resolution", 0.25);
    const std::string   occ_ndt_3d_map_topic           = nh_.param<std::string>("occ_ndt_3d_map_topic", "/map/3d/occ_ndt");
    const double        occ_ndt_3d_map_pub_rate        = nh_.param<double>("occ_ndt_3d_map_pub_rate", 10.0);

    std::vector<std::string> lasers2d, lasers3d;
    if(!nh_.getParam("lasers_2d", lasers2d) && !nh_.getParam("lasers_3d", lasers3d)) {
        ROS_ERROR_STREAM("Did not find any laser inputs!");
        return false;
    }

    std::string map_frame     = nh_.param<std::string>("map_frame", "/odom");

    node_rate_                = nh_.param<double>("rate", 0.0);
    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");

    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");
    tf_timeout_               = ros::Duration(nh_.param("tf_timeout", 0.1));

    linear_interval_[0]       = static_cast<float>(nh_.param<double>("range_min", 0.05));
    linear_interval_[1]       = static_cast<float>(nh_.param<double>("range_max", 30.0));
    angular_interval_[0]      = static_cast<float>(nh_.param<double>("angle_min",-M_PI));
    angular_interval_[1]      = static_cast<float>(nh_.param<double>("angle_max", M_PI));


    cslibs_gridmaps::utility::InverseModel inverse_model(occ_map_prob_prior, occ_map_prob_free, occ_map_prob_occ);
    occ_2d_mapper_.map_frame_ = map_frame;
    occ_2d_mapper_.mapper_.reset(
                new occ_map_2d_t(inverse_model,
                                 occ_2d_grid_resolution,
                                 occ_2d_grid_chunk_resolution,
                                 occ_2d_mapper_.map_frame_));
    occ_2d_mapper_.setCallback();

    ndt_2d_mapper_.map_frame_ = map_frame;
    ndt_2d_mapper_.mapper_.reset(
                new ndt_map_2d_t(ndt_2d_grid_resolution,
                                 ndt_2d_sampling_resolution,
                                 ndt_2d_mapper_.map_frame_));
    ndt_2d_mapper_.setCallback();

    occ_ndt_2d_mapper_.map_frame_ = map_frame;
    occ_ndt_2d_mapper_.mapper_.reset(
                new occ_ndt_map_2d_t(inverse_model,
                                     occ_ndt_2d_grid_resolution,
                                     occ_ndt_2d_sampling_resolution,
                                     occ_ndt_2d_mapper_.map_frame_));
    occ_ndt_2d_mapper_.setCallback();

    ndt_3d_mapper_.map_frame_ = map_frame;
    ndt_3d_mapper_.mapper_.reset(
                new ndt_map_3d_t(ndt_3d_grid_resolution,
                                 ndt_3d_sampling_resolution,
                                 ndt_3d_mapper_.map_frame_));
    ndt_3d_mapper_.setCallback();

    occ_ndt_3d_mapper_.map_frame_ = map_frame;
    occ_ndt_3d_mapper_.mapper_.reset(
                new occ_ndt_map_3d_t(inverse_model,
                                     occ_ndt_3d_grid_resolution,
                                     occ_ndt_3d_sampling_resolution,
                                     occ_ndt_3d_mapper_.map_frame_));
    occ_ndt_3d_mapper_.setCallback();

    for(const auto &l : lasers2d) {
        ROS_INFO_STREAM("Subscribing to 2d laser '" << l << "'");
        sub_lasers_.emplace_back(nh_.subscribe(l,
                                               static_cast<unsigned int>(subscriber_queue_size),
                                               &MapperNode3d::laserscan2d,
                                               this));
    }

    for(const auto &l : lasers3d) {
        ROS_INFO_STREAM("Subscribing to 3d laser '" << l << "'");
        sub_lasers_.emplace_back(nh_.subscribe(l,
                                               static_cast<unsigned int>(subscriber_queue_size),
                                               &MapperNode3d::laserscan3d,
                                               this));
    }

    ros::Time now = ros::Time::now();
    occ_2d_mapper_.     setup(nh_, occ_2d_map_topic,     occ_2d_map_pub_rate,     now);
    ndt_2d_mapper_.     setup(nh_, ndt_2d_map_topic,     ndt_2d_map_pub_rate,     now);
    occ_ndt_2d_mapper_. setup(nh_, occ_ndt_2d_map_topic, occ_ndt_2d_map_pub_rate, now);
    ndt_3d_mapper_.     setup(nh_, ndt_3d_map_topic,     ndt_3d_map_pub_rate,     now);
    occ_ndt_3d_mapper_. setup(nh_, occ_ndt_3d_map_topic, occ_ndt_3d_map_pub_rate, now);

    tf_.reset(new cslibs_math_ros::tf::TFListener2d);

    ROS_INFO_STREAM("Setup succesful!");
    return true;
}

void MapperNode3d::run()
{
    ros::Rate r(node_rate_);
    while(ros::ok()) {
        const ros::Time now = ros::Time::now();
        occ_2d_mapper_.    requestMap(now);
        ndt_2d_mapper_.    requestMap(now);
        occ_ndt_2d_mapper_.requestMap(now);
        ndt_3d_mapper_.    requestMap(now);
        occ_ndt_3d_mapper_.requestMap(now);
        r.sleep();
        ros::spinOnce();
    }
}

void MapperNode3d::laserscan2d(
        const sensor_msgs::LaserScanConstPtr & msg)
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

    insert(occ_2d_mapper_,     msg->header.frame_id, msg->header.stamp, laserscan);
    insert(ndt_2d_mapper_,     msg->header.frame_id, msg->header.stamp, laserscan);
    insert(occ_ndt_2d_mapper_, msg->header.frame_id, msg->header.stamp, laserscan);
}

void MapperNode3d::laserscan3d(
        const sensor_msgs::PointCloud2ConstPtr & msg)
{
    pcl::PointCloud<pcl::PointXYZI> laserscan;
    std::vector<int> indices;
    pcl::fromROSMsg(*msg, laserscan);
    pcl::removeNaNFromPointCloud(laserscan, laserscan, indices);

    insert<ndt_map_3d_t,     pcl::PointXYZI>(
                ndt_3d_mapper_,     msg->header.frame_id, msg->header.stamp,  laserscan.makeShared());
    insert<occ_ndt_map_3d_t, pcl::PointXYZI>(
                occ_ndt_3d_mapper_, msg->header.frame_id, msg->header.stamp,  laserscan.makeShared());
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cslibs_mapping_node_3d");
    cslibs_mapping::MapperNode3d instance;
    instance.setup();
    instance.run();

    return 0;
}
