#include "mapper_node_2d.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <cslibs_math_2d/linear/polar_pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>
#include <cslibs_math_ros/geometry_msgs/conversion_2d.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

/// OUTPUT
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <Board.h>

namespace cslibs_mapping {
MapperNode2d::MapperNode2d() :
    nh_("~")
{
}

bool MapperNode2d::setup()
{
    ROS_INFO_STREAM("Setting up subscribers");
    const std::string   path_topic                  = nh_.param<std::string>("path_topic", "/map/path");
    const double        path_pub_rate               = nh_.param<double>     ("path_pub_rate", 10.0);
    const double        path_update_rate            = nh_.param<double>     ("path_update_rate", 10.0);
    const double        occ_grid_resolution         = nh_.param<double>     ("occ_map_resolution", 0.05);
    const double        occ_grid_chunk_resolution   = nh_.param<double>     ("occ_map_chunk_resolution", 5.0);
    const std::string   occ_map_topic               = nh_.param<std::string>("occ_map_topic", "/map/occ");
    const double        occ_map_pub_rate            = nh_.param<double>     ("occ_map_pub_rate",  10.0);
    const double        occ_map_prob_free           = nh_.param<double>     ("occ_map_prob_free", 0.45);
    const double        occ_map_prob_occ            = nh_.param<double>     ("occ_map_prob_occ",  0.55);
    const double        occ_map_prob_prior          = nh_.param<double>     ("occ_map_prob_prior", 0.5);

    const std::string   ndt_map_topic               = nh_.param<std::string>("ndt_map_topic", "/map/ndt");
    const double        ndt_map_pub_rate            = nh_.param<double>     ("ndt_map_pub_rate", 10.0);
    const double        ndt_grid_resolution         = nh_.param<double>     ("ndt_grid_resolution", 1.0);
    const double        ndt_sampling_resolution     = nh_.param<double>     ("ndt_sampling_resolution", 0.05);

    std::vector<std::string> laser_topics;
    if(!nh_.getParam("laser_topics", laser_topics)) {
        ROS_ERROR_STREAM("Did not find any laser topics.");
        return false;
    }
    std::vector<int> laser_queue_sizes;
    if(!nh_.getParam("laser_queue_sizes", laser_queue_sizes)) {
        ROS_ERROR_STREAM("Did not find any laser queue sizes.");
        return false;
    }
    if(laser_topics.size() != laser_queue_sizes.size()) {
        ROS_ERROR_STREAM("Laser topics count and queue sizes have to match.");
        return false;
    }

    map_frame_                = nh_.param<std::string>                      ("map_frame", "/odom");

    node_rate_                = nh_.param<double>                           ("rate", 0.0);
    undistortion_             = nh_.param<bool>                             ("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>                      ("undistortion_fixed_frame", "/odom");

    tf_timeout_               = ros::Duration(nh_.param("tf_timeout", 0.1));

    laser_linear_interval_[0]  = static_cast<float>(nh_.param<double>("range_min", 0.05));
    laser_linear_interval_[1]  = static_cast<float>(nh_.param<double>("range_max", 30.0));
    laser_angular_interval_[0] = static_cast<float>(nh_.param<double>("angle_min",-M_PI));
    laser_angular_interval_[1] = static_cast<float>(nh_.param<double>("angle_max", M_PI));


    cslibs_gridmaps::utility::InverseModel inverse_model(occ_map_prob_prior, occ_map_prob_free, occ_map_prob_occ);
    occ_mapper_.reset(new OccupancyGridMapper2d(inverse_model,
                                                occ_grid_resolution,
                                                occ_grid_chunk_resolution,
                                                map_frame_));
    occ_mapper_->setCallback(OccupancyGridMapper2d::callback_t::from<MapperNode2d, &MapperNode2d::publishOcc>(this));

    ndt_mapper_.reset(new NDTGridMapper2d(ndt_grid_resolution, ndt_sampling_resolution, map_frame_));
    ndt_mapper_->setCallback(OccupancyGridMapper2d::callback_t::from<MapperNode2d, &MapperNode2d::publishNDT>(this));

    for(std::size_t i = 0 ; i < laser_topics.size() ; ++i) {
        const auto &l = laser_topics[i];
        const unsigned int s = static_cast<unsigned int>(laser_queue_sizes[i]);
        ROS_INFO_STREAM("Subscribing to laser '" << l << "'");
        sub_lasers_.emplace_back(nh_.subscribe(l,
                                               s,
                                               &MapperNode2d::laserscan,
                                               this));
    }

    pub_path_               = nh_.advertise<nav_msgs::Path>(path_topic, 1);
    pub_path_interval_      = ros::Duration(path_pub_rate > 0.0 ? 1.0 / path_pub_rate : 0.0);
    pub_path_last_time_     = ros::Time::now();
    path_.header.frame_id   = map_frame_;
    path_.header.stamp      = ros::Time::now();
    path_update_interval_   = ros::Duration(path_update_rate > 0.0 ? 1.0 / path_update_rate : 0.0);

    pub_occ_map_            = nh_.advertise<nav_msgs::OccupancyGrid>(occ_map_topic, 1);
    pub_occ_interval_       = ros::Duration(occ_map_pub_rate > 0.0 ? 1.0 / occ_map_pub_rate : 0.0);
    pub_occ_last_time_      = ros::Time::now();
    occ_free_threshold_     = nh_.param<double>("occ_free_threshold", 0.196);   // values retrieved from ros map_server documentation
    occ_occupied_threshold_ = nh_.param<double>("occ_occupied_treshold", 0.65); // values retrieved from ros map_server documentation

    pub_ndt_map_            = nh_.advertise<nav_msgs::OccupancyGrid>(ndt_map_topic, 1);
    pub_ndt_interval_       = ros::Duration(occ_map_pub_rate > 0.0 ? 1.0 / ndt_map_pub_rate : 0.0);
    pub_ndt_last_time_      = ros::Time::now();

    service_save_map_       = nh_.advertiseService(nh_.getNamespace() + "/save_map", &MapperNode2d::saveMap, this);

    tf_.reset(new cslibs_math_ros::tf::TFListener2d);

    ROS_INFO_STREAM("Setup succesful!");
    return true;
}

void MapperNode2d::run()
{
    if(node_rate_ == 0.0) {
        ros::spin();
    } else {
        ros::Rate r(node_rate_);
        while(ros::ok()) {
            r.sleep();
            ros::spinOnce();
        }
    }
}

void MapperNode2d::laserscan(const sensor_msgs::LaserScanConstPtr &msg)
{
    cslibs_math_2d::PolarPointlcoud2d::Ptr laserscan;
    if(undistortion_) {
        cslibs_math_ros::sensor_msgs::conversion_2d::from(msg,
                                                          laser_linear_interval_, laser_angular_interval_,
                                                          undistortion_fixed_frame_, tf_timeout_, *tf_,
                                                          laserscan);
    }
    if(!laserscan) {
        cslibs_math_ros::sensor_msgs::conversion_2d::from(msg, laser_linear_interval_, laser_angular_interval_, laserscan);
    }
    if(!laserscan)
        return;

    cslibs_math_2d::Transform2d o_T_l;
    cslibs_time::TimeFrame time_frame = cslibs_math_ros::sensor_msgs::conversion_2d::from(msg);
    if(tf_->lookupTransform(map_frame_, msg->header.frame_id,
                            ros::Time(time_frame.end.seconds()),
                            o_T_l,
                            tf_timeout_)) {

        {
            if(path_update_interval_.isZero() || (path_.header.stamp + path_update_interval_ < msg->header.stamp)) {
                path_.header.stamp = msg->header.stamp;
                geometry_msgs::PoseStamped p;
                p.pose = cslibs_math_ros::geometry_msgs::conversion_2d::from(o_T_l);
                p.header = path_.header;
                path_.poses.emplace_back(p);
            }

        }
        cslibs_math_2d::Pointcloud2d::Ptr points(new cslibs_math_2d::Pointcloud2d);
        measurement_t  m(points, o_T_l, time_frame.end);
        for(auto it = laserscan->begin() ; it != laserscan->end() ; ++it) {
            if(it->isNormal())
                points->insert(it->getCartesian());
        }
        occ_mapper_->insert(m);
        ndt_mapper_->insert(m);
    }

    publish();
}

void MapperNode2d::publish()
{
    const ros::Time now = ros::Time::now();
    if(pub_occ_last_time_.isZero())
        pub_occ_last_time_ = now;

    if(pub_occ_interval_.isZero() || (pub_occ_last_time_ + pub_occ_interval_ < now)) {
        occ_mapper_->requestMap();
    }
    if(pub_ndt_interval_.isZero() || (pub_ndt_last_time_ + pub_ndt_interval_ < now)) {
        ndt_mapper_->requestMap();
    }
    if(pub_path_interval_.isZero() || (pub_path_last_time_ + pub_path_interval_ < now)) {
        pub_path_.publish(path_);
        pub_path_last_time_ = now;
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

bool MapperNode2d::saveMap(cslibs_mapping::SaveMap::Request &req,
                           cslibs_mapping::SaveMap::Response &)
{
    return saveMap(req.path.data);
}

bool MapperNode2d::saveMap(const std::string &path)
{
    boost::filesystem::path p(path);

    if(!boost::filesystem::is_directory(p)) {
        ROS_ERROR_STREAM("'" << path << "' is not a directory.");
        return false;
    }

    OccupancyGridMapper2d::static_map_stamped_t occ_map;
    occ_mapper_->get(occ_map);
    const std::size_t occ_height = occ_map.data()->getHeight();
    const std::size_t occ_width  = occ_map.data()->getWidth();


    std::string occ_path_yaml = (p / boost::filesystem::path("occ.map.yaml")).string();
    std::string occ_path_pgm  = (p / boost::filesystem::path("occ.map.pgm")).string();
    std::string occ_path_raw_pgm = (p / boost::filesystem::path("occ.map.raw.pgm")).string();
    {
        std::ofstream occ_out_yaml(occ_path_yaml);
        if(!occ_out_yaml.is_open()) {
            ROS_ERROR_STREAM("Could not open file '" << occ_path_yaml << "'.");
            return false;
        }
        /// write occupancy map meta data
        YAML::Emitter occ_yaml(occ_out_yaml);
        occ_yaml << YAML::BeginMap;
        occ_yaml << "image" << occ_path_pgm;
        occ_yaml << "resolution" << occ_map.data()->getResolution();
        occ_yaml << "origin" << YAML::BeginSeq;
        const transform_t origin = occ_map.data()->getOrigin();
        occ_yaml << origin.tx() << origin.ty() << origin.yaw();
        occ_yaml << YAML::EndSeq;
        occ_yaml << "oocupied_threshold" << occ_occupied_threshold_;
        occ_yaml << "free_thresh" << occ_free_threshold_;
        occ_yaml << "negate" << 0;
        occ_yaml << YAML::EndMap;
    }
    {
        std::ofstream occ_out_pgm(occ_path_pgm);
        std::ofstream occ_out_raw_pgm(occ_path_raw_pgm);
        if(!occ_out_pgm.is_open()) {
            ROS_ERROR_STREAM("Could not open file '" << occ_path_pgm << "'.");
            return false;
        }
        if(!occ_out_raw_pgm.is_open()) {
            ROS_ERROR_STREAM("Could not open file '" << occ_path_raw_pgm << "'.");
            return false;
        }

        const auto &occ_data = occ_map.data()->getData();
        const double *occ_data_ptr = occ_data.data();
        const std::size_t occ_max_idx = occ_width - 1ul;

        /// write pgm headers
        occ_out_pgm << "P5 \n";
        occ_out_pgm << "# CREATOR: cslibs_mapping_node_2d " << occ_map.data()->getResolution() << "m/pix \n";
        occ_out_pgm << occ_width << " " << occ_height << "\n";
        occ_out_pgm << 255 << "\n";
        occ_out_raw_pgm << "P5 \n";
        occ_out_raw_pgm << "# CREATOR: cslibs_mapping_node_2d " << occ_map.data()->getResolution() << "m/pix \n";
        occ_out_raw_pgm << occ_width << " " << occ_height << "\n";
        occ_out_raw_pgm << 255 << "\n";

        auto convert_ros = [this](const double p) {
            if(cslibs_math::common::le(p, occ_free_threshold_))
                return static_cast<uint8_t>(254);
            if(cslibs_math::common::le(p,occ_occupied_threshold_))
                return static_cast<uint8_t>(0);
            return static_cast<uint8_t>(205);
        };
        auto convert_raw = [](const double p) {
            return static_cast<uint8_t>((1.0 - p) * 255);
        };
        for(std::size_t i = 0 ; i < occ_height ; ++i) {
            for(std::size_t j = 0 ; j < occ_max_idx; ++j) {
                occ_out_pgm << convert_ros(*occ_data_ptr);
                occ_out_raw_pgm << convert_raw(*occ_data_ptr);
                ++occ_data_ptr;
            }
            occ_out_pgm << convert_ros(*occ_data_ptr);
            occ_out_raw_pgm << convert_raw(*occ_data_ptr);
            ++occ_data_ptr;
        }

        occ_out_pgm.close();
        occ_out_raw_pgm.close();
    }
    /// save map stuff here
#if CSLIBS_MAPPING_LIBBOARD
    {
        std::string occ_path_eps = (p / boost::filesystem::path("occ.map.eps")).string();
        LibBoard::Board occ_board;
        LibBoard::Image occ_board_img(occ_path_raw_ppm.c_str(), 0, 0, occ_width, occ_height);
        occ_board.insert(occ_board_img, 0);
        occ_board.setLineWidth(0.1);
        occ_board.setPenColorRGBi( 0, 255, 0 );
        occ_board.setLineCap(LibBoard::Shape::RoundCap);
        const double occ_inv_resolution = 1.0 / occ_map.data()->getResolution();

        const double occ_inv_resolution = 1.0 / occ_map.data()->getResolution();
        const cslibs_math_2d::Transform2d m_t_w = occ_map.data()->getOrigin().inverse();
        for(std::size_t i = 1 ; i < path_.poses.size(); ++i) {
            const cslibs_math_2d::Transform2d t0 = m_t_w * cslibs_math_ros::geometry_msgs::conversion_2d::from(path_.poses[i-1].pose);
            const cslibs_math_2d::Transform2d t1 = m_t_w * cslibs_math_ros::geometry_msgs::conversion_2d::from(path_.poses[i].pose);
            occ_board.drawLine(t0.tx() * occ_inv_resolution, t0.ty() * occ_inv_resolution,
                               t1.tx() * occ_inv_resolution, t1.ty() * occ_inv_resolution,
                               1);
            occ_board.drawCircle(t0.tx() * occ_inv_resolution, t0.ty() * occ_inv_resolution, 0.5, 2);
        }
        occ_board.saveEPS(occ_path_eps.c_str());
    }
#endif
    return true;
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
