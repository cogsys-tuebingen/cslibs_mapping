#include "ndt_map_loader_3d.h"

#include <cslibs_ndt_3d/conversion/pointcloud.hpp>

namespace cslibs_mapping {
NDTMapLoader3D::NDTMapLoader3D() :
    nh_("~")
{
}

NDTMapLoader3D::~NDTMapLoader3D()
{
}

void NDTMapLoader3D::run()
{
    std::cerr << "Starting to set up." << std::endl;
    if(!setup()) {
        std::cerr << "Cannot setup the map loader node." << std::endl;
        ros::shutdown();
    }
    std::cerr << "Setup succesful." << std::endl;

    ros::spin();
}

bool NDTMapLoader3D::setup()
{
    const std::string topic_ndt_3d      = nh_.param<std::string>("topic_ndt_3d", "/map/3d/ndt");
    const std::string topic_occ_ndt_3d  = nh_.param<std::string>("topic_occ_ndt_3d", "/map/3d/occ_ndt");
    const std::string path_ndt_3d       = nh_.param<std::string>("path_ndt_3d", "");
    const std::string path_occ_ndt_3d   = nh_.param<std::string>("path_occ_ndt_3d", "");


    pub_ndt_3d_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_ndt_3d, 1);
    pub_occ_ndt_3d_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_occ_ndt_3d, 1);

    if(path_ndt_3d != "") {
        if(!cslibs_ndt_3d::dynamic_maps::loadBinary(path_ndt_3d, map_ndt_)) {
            std::cerr << "Could not load ndt 3d map '" << path_ndt_3d << "'." << std::endl;
            return false;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr points;
        cslibs_ndt_3d::conversion::from(map_ndt_, points);
        map_ndt_distributions_.reset(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*points, *map_ndt_distributions_);
        map_ndt_distributions_->header.frame_id = "/map";

    }
    if(path_occ_ndt_3d != "") {
        if(!cslibs_ndt_3d::dynamic_maps::loadBinary(path_occ_ndt_3d, map_occ_ndt_)) {
            std::cerr << "Could not load occupancy ndt 3d map '" << path_occ_ndt_3d << "'." << std::endl;
            return false;
        }
        cslibs_gridmaps::utility::InverseModel::Ptr ivm(new cslibs_gridmaps::utility::InverseModel(0.5, 0.45, 0.65));
        pcl::PointCloud<pcl::PointXYZI>::Ptr points;
        cslibs_ndt_3d::conversion::from(map_occ_ndt_, points, ivm);
        map_occ_ndt_distributions_.reset(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*points, *map_occ_ndt_distributions_);
        map_occ_ndt_distributions_->header.frame_id = "/map";
    }

    service_ = nh_.advertiseService(nh_.getNamespace() + "/resend", &NDTMapLoader3D::resend, this);

    return true;
}

bool NDTMapLoader3D::resend(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res)
{
    if(!map_ndt_distributions_ && !map_occ_ndt_distributions_) {
        ROS_ERROR_STREAM("What can I say, I have nothing to offer!");
        return false;
    }

    if(map_ndt_distributions_) {
        map_ndt_distributions_->header.stamp = ros::Time::now();
        pub_ndt_3d_.publish(map_ndt_distributions_);
    }
    if(map_occ_ndt_distributions_) {
        map_occ_ndt_distributions_->header.stamp = ros::Time::now();
        pub_occ_ndt_3d_.publish(map_occ_ndt_distributions_);
    }

    return true;
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "NDTMapLoader3D");

    cslibs_mapping::NDTMapLoader3D ml;
    ml.run();

    return 0;
}
