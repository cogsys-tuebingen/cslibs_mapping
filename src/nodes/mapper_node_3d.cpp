#include "mapper_node_3d.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <cslibs_time/time.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>
#include <cslibs_math_ros/geometry_msgs/conversion_2d.hpp>

//#include <ndt_map/NDTMapMsg.h>
//#include <ndt_map/ndt_conversions.h>
//#include <pcl/common/transforms.h>

namespace cslibs_mapping {
MapperNode3d::MapperNode3d() :
    nh_("~")
{ }

MapperNode3d::~MapperNode3d()
{
    if (output_path_ != "") {
        std::cout << "Saving Maps..." << std::endl;
        const bool success = saveMap(output_path_);
        std::cout << "Saving Maps was " << (success ? "successful :-)." : "unsuccessful :-(.") << std::endl;
    } else
      std::cout << "Closing without Saving Maps..." << std::endl;

    if (ndt_3d_map_oru_)
        delete ndt_3d_map_oru_;
}

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
    const bool          occ_2d_map_active              = nh_.param<bool>("occ_2d_map_active", true);

    const double        ndt_2d_grid_resolution         = nh_.param<double>("ndt_2d_grid_resolution", 1.0);
    const double        ndt_2d_sampling_resolution     = nh_.param<double>("ndt_2d_sampling_resolution", 0.025);
    const std::string   ndt_2d_map_topic               = nh_.param<std::string>("ndt_2d_map_topic", "/map/2d/ndt");
    const double        ndt_2d_map_pub_rate            = nh_.param<double>("ndt_2d_map_pub_rate", 10.0);
    const bool          ndt_2d_map_active              = nh_.param<bool>("ndt_2d_map_active", true);

    const double        occ_ndt_2d_grid_resolution     = nh_.param<double>("occ_ndt_2d_grid_resolution", 1.0);
    const double        occ_ndt_2d_sampling_resolution = nh_.param<double>("occ_ndt_2d_sampling_resolution", 0.025);
    const std::string   occ_ndt_2d_map_topic           = nh_.param<std::string>("occ_ndt_2d_map_topic", "/map/2d/occ_ndt");
    const double        occ_ndt_2d_map_pub_rate        = nh_.param<double>("occ_ndt_2d_map_pub_rate", 10.0);
    const bool          occ_ndt_2d_map_active          = nh_.param<bool>("occ_ndt_2d_map_active", true);

    const double        occ_3d_grid_resolution         = nh_.param<double>("occ_3d_grid_resolution", 1.0);
    const std::string   occ_3d_map_topic               = nh_.param<std::string>("occ_3d_map_topic", "/map/3d/occ");
    const double        occ_3d_map_pub_rate            = nh_.param<double>("occ_3d_map_pub_rate", 10.0);
    const bool          occ_3d_map_active              = nh_.param<bool>("occ_3d_map_active", true);

    const double        ndt_3d_grid_resolution         = nh_.param<double>("ndt_3d_grid_resolution", 2.0);
    const std::string   ndt_3d_map_topic               = nh_.param<std::string>("ndt_3d_map_topic", "/map/3d/ndt");
    const double        ndt_3d_map_pub_rate            = nh_.param<double>("ndt_3d_map_pub_rate", 10.0);
    const bool          ndt_3d_map_active              = nh_.param<bool>("ndt_3d_map_active", true);

    const double        occ_ndt_3d_grid_resolution     = nh_.param<double>("occ_ndt_3d_grid_resolution", 2.0);
    const std::string   occ_ndt_3d_map_topic           = nh_.param<std::string>("occ_ndt_3d_map_topic", "/map/3d/occ_ndt");
    const double        occ_ndt_3d_map_pub_rate        = nh_.param<double>("occ_ndt_3d_map_pub_rate", 10.0);
    const bool          occ_ndt_3d_map_active          = nh_.param<bool>("occ_ndt_3d_map_active", true);

    const std::string   path_topic                     = nh_.param<std::string>("path_topic", "/map/path");
    const double        path_update_rate               = nh_.param<double>("path_update_rate", 10.0);

    filter_laserscan3d_    = nh_.param<bool>("filter_laserscan3d", false);
    filter_size_           = nh_.param<double>("filter_size", 0.05);
//    ndt_3d_map_oru_active_ = nh_.param<bool>("ndt_3d_map_oru_active", true);

    std::vector<std::string> lasers2d, lasers3d;
    if (!nh_.getParam("lasers_2d", lasers2d))
        ROS_ERROR_STREAM("Did not find any 2d laser inputs!");
    if (!nh_.getParam("lasers_3d", lasers3d))
        ROS_ERROR_STREAM("Did not find any 3d laser inputs!");
    if (lasers2d.empty() && lasers3d.empty())
        return false;

    map_frame_                = nh_.param<std::string>("map_frame", "/odom");
    base_frame_               = nh_.param<std::string>("base_frame", "/base_link");
    output_path_              = nh_.param<std::string>("output_path", "");

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
    occ_2d_mapper_.map_frame_ = map_frame_;
    occ_2d_mapper_.mapper_.reset(
                new occ_map_2d_t(inverse_model,
                                 occ_2d_grid_resolution,
                                 occ_2d_grid_chunk_resolution,
                                 occ_2d_mapper_.map_frame_));
    occ_2d_mapper_.setCallback();

    ndt_2d_mapper_.map_frame_ = map_frame_;
    ndt_2d_mapper_.mapper_.reset(
                new ndt_map_2d_t(ndt_2d_grid_resolution,
                                 ndt_2d_sampling_resolution,
                                 ndt_2d_mapper_.map_frame_));
    ndt_2d_mapper_.setCallback();

    occ_ndt_2d_mapper_.map_frame_ = map_frame_;
    occ_ndt_2d_mapper_.mapper_.reset(
                new occ_ndt_map_2d_t(inverse_model,
                                     occ_ndt_2d_grid_resolution,
                                     occ_ndt_2d_sampling_resolution,
                                     occ_ndt_2d_mapper_.map_frame_));
    occ_ndt_2d_mapper_.setCallback();

    occ_3d_mapper_.map_frame_ = map_frame_;
    occ_3d_mapper_.mapper_.reset(
                new occ_map_3d_t(inverse_model,
                                 occ_3d_grid_resolution,
                                 occ_3d_mapper_.map_frame_));
    occ_3d_mapper_.setCallback();

    ndt_3d_mapper_.map_frame_ = map_frame_;
    ndt_3d_mapper_.mapper_.reset(
                new ndt_map_3d_t(ndt_3d_grid_resolution,
                                 ndt_3d_mapper_.map_frame_));
    ndt_3d_mapper_.setCallback();

    occ_ndt_3d_mapper_.map_frame_ = map_frame_;
    occ_ndt_3d_mapper_.mapper_.reset(
                new occ_ndt_map_3d_t(inverse_model,
                                     occ_ndt_3d_grid_resolution,
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
    occ_2d_mapper_.     setup(nh_, occ_2d_map_topic,     occ_2d_map_pub_rate,     now, occ_2d_map_active);
    ndt_2d_mapper_.     setup(nh_, ndt_2d_map_topic,     ndt_2d_map_pub_rate,     now, ndt_2d_map_active);
    occ_ndt_2d_mapper_. setup(nh_, occ_ndt_2d_map_topic, occ_ndt_2d_map_pub_rate, now, occ_ndt_2d_map_active);
    occ_3d_mapper_.     setup(nh_, occ_3d_map_topic,     occ_3d_map_pub_rate,     now, occ_3d_map_active);
    ndt_3d_mapper_.     setup(nh_, ndt_3d_map_topic,     ndt_3d_map_pub_rate,     now, ndt_3d_map_active);
    occ_ndt_3d_mapper_. setup(nh_, occ_ndt_3d_map_topic, occ_ndt_3d_map_pub_rate, now, occ_ndt_3d_map_active);

//    ndt_3d_map_oru_     = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(occ_ndt_3d_grid_resolution / 2.0));
//    ndt_3d_map_oru_pub_ = nh_.advertise<ndt_map::NDTMapMsg>("map/3d/ndt_oru", 1);

    path_update_interval_  = ros::Duration(path_update_rate > 0.0 ? 1.0 / path_update_rate : 0.0);
    path_.header.stamp     = now;
    path_.header.frame_id  = map_frame_;
    path_.header.stamp     = now;

    pub_path_         = nh_.advertise<nav_msgs::Path>(path_topic, 1);
    service_save_map_ = nh_.advertiseService(nh_.getNamespace() + "/save_map", &MapperNode3d::saveMap, this);

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
        occ_3d_mapper_.    requestMap(now);
        ndt_3d_mapper_.    requestMap(now);
        occ_ndt_3d_mapper_.requestMap(now);
        pub_path_.publish(path_);
        r.sleep();
        ros::spinOnce();
    }
}

void MapperNode3d::laserscan2d(
        const sensor_msgs::LaserScanConstPtr & msg)
{std::cout << "Laser 2D: " << msg->header.frame_id << std::endl;
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
{std::cout << "Laser 3D: " << msg->header.frame_id << std::endl;
    pcl::PointCloud<pcl::PointXYZ> laserscan;
    std::vector<int> indices;
    pcl::fromROSMsg(*msg, laserscan);
    pcl::removeNaNFromPointCloud(laserscan, laserscan, indices);

    if (filter_laserscan3d_) {
        pcl::PointCloud<pcl::PointXYZ> laserscan_copy = laserscan;
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize(filter_size_, filter_size_, filter_size_);
        filter.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(laserscan_copy));
        filter.filter(laserscan);
    }

    insert<occ_map_3d_t,     octomap_msg_3d_t, pcl::PointXYZ>(
                occ_3d_mapper_,     msg->header.frame_id, msg->header.stamp, laserscan.makeShared());
    insert<ndt_map_3d_t,     msg_3d_t,         pcl::PointXYZ>(
                ndt_3d_mapper_,     msg->header.frame_id, msg->header.stamp, laserscan.makeShared());
    insert<occ_ndt_map_3d_t, msg_3d_t,         pcl::PointXYZ>(
                occ_ndt_3d_mapper_, msg->header.frame_id, msg->header.stamp, laserscan.makeShared());
/*
    // Örebro NDT-OM Stuff
    if (ndt_3d_map_oru_active_ && ndt_3d_map_oru_) {
        tf::Transform o_T_l;
        if (tf_->lookupTransform(map_frame_,
                                 msg->header.frame_id,
                                 msg->header.stamp,
                                 o_T_l,
                                 tf_timeout_)) {

            transform_3d_t origin = cslibs_math_ros::tf::conversion_3d::from(o_T_l);

            pcl::PointCloud<pcl::PointXYZ> pc, pcc;
            pcl::copyPointCloud(laserscan, pc);

            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            for (int i = 0 ; i < 3 ; ++ i)
                for (int j = 0 ; j < 3 ; ++ j)
                    transform.matrix()(i, j) = o_T_l.getBasis()[i][j];
            transform.translation() << o_T_l.getOrigin().x(), o_T_l.getOrigin().y(), o_T_l.getOrigin().z();
            pcl::transformPointCloud(pc, pcc, transform);

            cslibs_time::Time now = cslibs_time::Time::now();
            ndt_3d_map_oru_->addPointCloud(Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), pcc);
            ndt_3d_map_oru_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), 0.1);

            const double time_ms = (cslibs_time::Time::now() - now).milliseconds();
            std::cout << "[NDTOru]: Insertion took " << time_ms << "ms \n";

            oru_stats_ += time_ms;
            static const std::string filename = "/tmp/oru_stats";
            std::ofstream out;
            out.open(filename, std::ofstream::out | std::ofstream::app);
            out << oru_stats_.getN() << " | " << time_ms << " | " << oru_stats_.getMean() << " | " << oru_stats_.getStandardDeviation() << "\n";
            out.close();

//            ndt_map::NDTMapMsg map_msg;
//            lslgeneric::toMessage(ndt_3d_map_oru_, map_msg, map_frame_);
//            ndt_3d_map_oru_pub_.publish(map_msg);
        }
    }*/
}

bool MapperNode3d::saveMap(
    SaveMap::Request                  & req,
    cslibs_mapping::SaveMap::Response &)
{
    return saveMap(req.path.data);
}

bool MapperNode3d::saveMap(
    const std::string & path)
{
    boost::filesystem::path p(path);

    if(!boost::filesystem::is_directory(p))
      boost::filesystem::create_directories(p);
    if(!boost::filesystem::is_directory(p)) {
        std::cout << "[MapperNode3d]: '" << path << "' is not a directory." << std::endl;
        return false;
    }

    const bool res1 = occ_2d_mapper_.    mapper_->saveMap(path + occ_2d_mapper_.    pub_map_.getTopic() + "/", path_);
    const bool res2 = ndt_2d_mapper_.    mapper_->saveMap(path + ndt_2d_mapper_.    pub_map_.getTopic() + "/", path_);
    const bool res3 = occ_ndt_2d_mapper_.mapper_->saveMap(path + occ_ndt_2d_mapper_.pub_map_.getTopic() + "/", path_);
    const bool res4 = occ_3d_mapper_.    mapper_->saveMap(path + occ_3d_mapper_.    pub_map_.getTopic() + "/", path_);
    const bool res5 = ndt_3d_mapper_.    mapper_->saveMap(path + ndt_3d_mapper_.    pub_map_.getTopic() + "/", path_);
    const bool res6 = occ_ndt_3d_mapper_.mapper_->saveMap(path + occ_ndt_3d_mapper_.pub_map_.getTopic() + "/", path_);
    return res1 && res2 && res3 && res4 && res5 && res6;
}

void MapperNode3d::updatePath(
    const ros::Time & time)
{
    if(path_update_interval_.isZero() || (path_.header.stamp + path_update_interval_ < time)) {
        cslibs_math_2d::Transform2d o_T_b;
        if(tf_->lookupTransform(path_.header.frame_id, base_frame_,
                                time,
                                o_T_b,
                                tf_timeout_)) {
            path_.header.stamp = time;
            geometry_msgs::PoseStamped p;
            p.pose = cslibs_math_ros::geometry_msgs::conversion_2d::from(o_T_b);
            p.header = path_.header;
            path_.poses.emplace_back(p);
        }
    }
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
