#ifndef NDT_MAP_LOADER_3D_H
#define NDT_MAP_LOADER_3D_H

#include <cslibs_ndt_3d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>

namespace cslibs_mapping {
class NDTMapLoader3D
{
public:
    NDTMapLoader3D();
    virtual ~NDTMapLoader3D();

    void run();

private:
    ros::NodeHandle     nh_;
    ros::Publisher      pub_ndt_3d_;
    ros::Publisher      pub_occ_ndt_3d_;
    ros::ServiceServer  service_;

    cslibs_ndt_3d::dynamic_maps::Gridmap::Ptr          map_ndt_;
    cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::Ptr map_occ_ndt_;
    sensor_msgs::PointCloud2::Ptr                      map_ndt_distributions_;
    sensor_msgs::PointCloud2::Ptr                      map_occ_ndt_distributions_;

    bool setup();

    bool resend(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res);

};
}

#endif // NDT_MAP_LOADER_3D_H
