#include "mapper_node_3d.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <cslibs_time/time.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>
#include <cslibs_math_ros/geometry_msgs/conversion_2d.hpp>

//#ifdef WITH_ORU_NDT
#include <ndt_map/lazy_grid.h>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>
#include <pcl/common/transforms.h>
//#endif

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

    oru_thread_.join();
}

bool MapperNode3d::setup()
{std::cout << "SIZES: " << sizeof(cslibs_ndt::Distribution<2>) << "; " << sizeof(cslibs_ndt::OccupancyDistribution<3>) <<
              " - " << sizeof(cslibs_ndt::Distribution<2>) << "; " << sizeof(cslibs_ndt::OccupancyDistribution<3>) <<
              " - " << sizeof(cslibs_math::statistics::Distribution<2, 3>) << "; " << sizeof(cslibs_math::statistics::Distribution<3, 3>) << std::endl;
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
//#ifdef WITH_ORU_NDT
    ndt_3d_map_oru_active_      = nh_.param<bool>("ndt_3d_map_oru_active", true);
    //ndt_2d_map_oru_active_      = nh_.param<bool>("ndt_2d_map_oru_active", true);
    ndt_3d_map_oru_pub_active_  = nh_.param<bool>("ndt_3d_map_oru_pub_active", false);
    //ndt_2d_map_oru_pub_active_  = nh_.param<bool>("ndt_2d_map_oru_pub_active", false);
//#endif

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

    double visibility_threshold         = nh_.param<double>("visibility_threshold", 0.4);
    double prob_visible_if_occluded     = nh_.param<double>("prob_visible_if_occluded", 0.2);
    double prob_visible_if_not_occluded = nh_.param<double>("prob_visible_if_not_occluded", 0.8);

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

    cslibs_gridmaps::utility::InverseModel inverse_model_visibility(
                visibility_threshold, prob_visible_if_occluded, prob_visible_if_not_occluded);
    occ_ndt_3d_mapper_.map_frame_ = map_frame_;
    occ_ndt_3d_mapper_.mapper_.reset(
                new occ_ndt_map_3d_t(inverse_model,
                                     inverse_model_visibility,
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

//#ifdef WITH_ORU_NDT
    if (ndt_3d_map_oru_active_) {
    double ndt_oru_size_x = nh_.param<double>("ndt_oru_size_x", 0.0);
    double ndt_oru_size_y = nh_.param<double>("ndt_oru_size_y", 0.0);
    double ndt_oru_size_z = nh_.param<double>("ndt_oru_size_z", 0.0);
    double ndt_oru_cen_x  = nh_.param<double>("ndt_oru_cen_x",  0.0);
    double ndt_oru_cen_y  = nh_.param<double>("ndt_oru_cen_y",  0.0);
    double ndt_oru_cen_z  = nh_.param<double>("ndt_oru_cen_z",  0.0);
std::cout << "Oru: " << ndt_oru_cen_x << ", " << ndt_oru_size_x << std::endl;
    if (ndt_oru_size_x != 0.0 && ndt_oru_size_y != 0.0 && ndt_oru_size_z != 0.0)
        ndt_3d_map_oru_.reset(new lslgeneric::NDTMap(new lslgeneric::LazyGrid(occ_ndt_3d_grid_resolution / 2.0),
                                                     ndt_oru_cen_x, ndt_oru_cen_y, ndt_oru_cen_z,
                                                     ndt_oru_size_x, ndt_oru_size_y, ndt_oru_size_z, true));
    else
        ndt_3d_map_oru_.reset(new lslgeneric::NDTMap(new lslgeneric::LazyGrid(occ_ndt_3d_grid_resolution / 2.0)));

    ndt_3d_map_oru_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map/3d/ndt_oru", 1);
/*    if (ndt_oru_size_x != 0.0 && ndt_oru_size_y != 0.0 && ndt_oru_size_z != 0.0)
        ndt_2d_map_oru_.reset(new lslgeneric::NDTMap(new lslgeneric::LazyGrid(occ_ndt_2d_grid_resolution / 2.0),
                                                     ndt_oru_cen_x, ndt_oru_cen_y, ndt_oru_cen_z,
                                                     ndt_oru_size_x, ndt_oru_size_y, ndt_oru_size_z, true));
    else
        ndt_2d_map_oru_.reset(new lslgeneric::NDTMap(new lslgeneric::LazyGrid(occ_ndt_2d_grid_resolution / 2.0)));
    ndt_2d_map_oru_pub_ = nh_.advertise<ndt_map::NDTMapMsg>("/map/2d/ndt_oru", 1);
//#endif
*/}
    path_update_interval_  = ros::Duration(path_update_rate > 0.0 ? 1.0 / path_update_rate : 0.0);
    path_.header.stamp     = now;
    path_.header.frame_id  = map_frame_;
    path_.header.stamp     = now;

    pub_path_         = nh_.advertise<nav_msgs::Path>(path_topic, 1);
    service_save_map_ = nh_.advertiseService(nh_.getNamespace() + "/save_map", &MapperNode3d::saveMap, this);

    tf_.reset(new cslibs_math_ros::tf::TFListener2d);
    if (ndt_3d_map_oru_active_)
        oru_thread_ = std::thread([this](){publishOru3d();});

    ROS_INFO_STREAM("Setup succesful!");
    return true;
}

void MapperNode3d::publishOru3d()
{
    while (ros::ok()) {
        //#ifdef WITH_ORU_NDT
        // Örebro NDT-OM Stuff

        if (ndt_3d_map_oru_pub_active_) {
            pcl::PointCloud<pcl::PointXYZI> pcl;

            //ndt_3d_map_oru_->getGridSizeInMeters(map_msg.x_size, map_msg.y_size, map_msg.z_size);
            //ndt_3d_map_oru_->getCentroid(map_msg.x_cen, map_msg.y_cen, map_msg.z_cen);
            //ndt_3d_map_oru_->getCellSizeInMeters(map_msg.x_cell_size, map_msg.y_cell_size, map_msg.z_cell_size);
            for (const auto &c : ndt_3d_map_oru_->getAllInitializedCellsShared()) {
                lslgeneric::NDTCell &cell = *c;
                pcl::PointXYZI p;
                Eigen::Vector3d mean = cell.getMean();
                p.x = mean(0);
                p.y = mean(1);
                p.z = mean(2);
                p.intensity = cell.getOccupancyRescaled();
                if (std::isnormal(p.x) && std::isnormal(p.y) && std::isnormal(p.z) && std::isnormal(p.intensity))
                    pcl.push_back(p);
            }

            sensor_msgs::PointCloud2 map_msg;
            pcl::toROSMsg(pcl, map_msg);
            map_msg.header.stamp = ros::Time::now();
            map_msg.header.frame_id = map_frame_;
            ndt_3d_map_oru_pub_.publish(map_msg);
        }
        //#endif
        ros::Rate(10).sleep();
    }
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

/*
//#ifdef WITH_ORU_NDT
    // Örebro NDT-OM Stuff
    if (ndt_2d_map_oru_active_ && ndt_2d_map_oru_) {
        tf::Transform o_T_l;
        if (tf_->lookupTransform(map_frame_,
                                 msg->header.frame_id,
                                 msg->header.stamp,
                                 o_T_l,
                                 tf_timeout_)) {

            transform_3d_t origin = cslibs_math_ros::tf::conversion_3d::from(o_T_l);
            pcl::PointCloud<pcl::PointXYZ> pc, pcc;
            for(auto it = laserscan->begin() ; it != laserscan->end() ; ++ it)
                if(it->isNormal()) {
                    pcl::PointXYZ pp(it->getCartesian()(0),
                                     it->getCartesian()(1),
                                     0.1+0.02 * (double)rand()/(double)RAND_MAX);
                    pc.push_back(pp);
                }

            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            for (int i = 0 ; i < 3 ; ++ i)
                for (int j = 0 ; j < 3 ; ++ j)
                    transform.matrix()(i, j) = o_T_l.getBasis()[i][j];
            transform.translation() << o_T_l.getOrigin().x(), o_T_l.getOrigin().y(), o_T_l.getOrigin().z();
            pcl::transformPointCloud(pc, pcc, transform);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(pcc, pcc, indices);

            cslibs_time::Time now = cslibs_time::Time::now();
            ndt_2d_map_oru_->addPointCloud(Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), pcc);
            ndt_2d_map_oru_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e9, 255, Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), 0.1);

            const double time_ms = (cslibs_time::Time::now() - now).milliseconds();
            std::cout << "[NDTOru2d]: Insertion took " << time_ms << "ms, ";
            std::cout << ndt_2d_map_oru_->getAllCellsShared().size() << "|"
                      << ndt_2d_map_oru_->getAllInitializedCellsShared().size() << "|"
                      << ndt_2d_map_oru_->numberOfActiveCells() << " cells"
                      << " * mem: " << sizeof(lslgeneric::NDTCell) << " + " << sizeof(lslgeneric::NDTMap)
                      << " size: " << ndt_2d_map_oru_->byteSize() << "\n";

            ndt_2d_map_oru_stats_ += time_ms;
            static const std::string filename = "/tmp/oru2d_stats";
            std::ofstream out;
            out.open(filename, std::ofstream::out | std::ofstream::app);
            out << ndt_2d_map_oru_stats_.getN() << " | " << time_ms << " | " << ndt_2d_map_oru_stats_.getMean()
                << " | " << ndt_2d_map_oru_stats_.getStandardDeviation()
                << " | mem: " << ndt_2d_map_oru_->byteSize() << "\n";
            out.close();

            if (ndt_2d_map_oru_pub_active_) {
                ndt_map::NDTMapMsg map_msg;
                map_msg.header.stamp = msg->header.stamp;
                map_msg.header.frame_id = map_frame_;
                ndt_2d_map_oru_->getGridSizeInMeters(map_msg.x_size, map_msg.y_size, map_msg.z_size);
                ndt_2d_map_oru_->getCentroid(map_msg.x_cen, map_msg.y_cen, map_msg.z_cen);
                ndt_2d_map_oru_->getCellSizeInMeters(map_msg.x_cell_size, map_msg.y_cell_size, map_msg.z_cell_size);
                for (const auto &c : ndt_2d_map_oru_->getAllInitializedCellsShared()) {
                    lslgeneric::NDTCell &cell = *c;
                    ndt_map::NDTCellMsg cell_msg;
                    Eigen::Vector3d mean = cell.getMean();
                    cell_msg.mean_x = mean(0);
                    cell_msg.mean_y = mean(1);
                    cell_msg.mean_z = mean(2);
                    cell_msg.occupancy = cell.getOccupancyRescaled();
                    for (int i = 0 ; i < 3 ; ++ i)
                        for (int j = 0 ; j < 3 ; ++ j)
                            cell_msg.cov_matrix.push_back(cell.getCov()(i, j));
                    cell_msg.N = cell.getN();
                    map_msg.cells.push_back(cell_msg);
                }

                ndt_2d_map_oru_pub_.publish(map_msg);
            }
        }
    }*/
//#endif
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
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(pcc, pcc, indices);

            cslibs_time::Time now = cslibs_time::Time::now();
            ndt_3d_map_oru_->addPointCloud(Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), pcc);
            ndt_3d_map_oru_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e9, 255, Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), 0.1);

            const double time_ms = (cslibs_time::Time::now() - now).milliseconds();
            std::cout << "[NDTOru3d]: Insertion took " << time_ms << "ms, ";

            std::cout << ndt_3d_map_oru_ << std::endl;
            /*std::cout << ndt_3d_map_oru_->getAllCellsShared().size() << "|"
                          << ndt_3d_map_oru_->getAllInitializedCellsShared().size() << "|"
                          << ndt_3d_map_oru_->numberOfActiveCells() << " cells"
                          << " * mem: " << sizeof(lslgeneric::NDTCell) << " + " << sizeof(lslgeneric::NDTMap)
                          << " size: " << ndt_3d_map_oru_->byteSize() << "\n";
    */
            ndt_3d_map_oru_stats_ += time_ms;
            static const std::string filename = "/tmp/oru3d_stats";
            std::ofstream out;
            out.open(filename, std::ofstream::out | std::ofstream::app);
            out << ndt_3d_map_oru_stats_.getN() << " | " << time_ms << " | " << ndt_3d_map_oru_stats_.getMean()
                << " | " << ndt_3d_map_oru_stats_.getStandardDeviation()
                << " | mem: " << ndt_3d_map_oru_->byteSize() << "\n";
            out.close();
        }
    }

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


//#ifdef WITH_ORU_NDT
//    std::string oru_2d = path + "/Oru2d.jff";
    std::string oru_3d = path + "/Oru3d.jff";
    const bool res7 = ndt_3d_map_oru_active_ && (ndt_3d_map_oru_->writeToJFF(oru_3d.c_str())) == 0;
    const bool res8 = true;//ndt_2d_map_oru_active_ && (ndt_2d_map_oru_->writeToJFF(oru_2d.c_str())) == 0;
    return res1 && res2 && res3 && res4 && res5 && res6 && res7 && res8;
//#else
//    return res1 && res2 && res3 && res4 && res5 && res6;
//#endif
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
