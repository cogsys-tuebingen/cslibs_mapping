#ifndef GRID_MAPPER_NODE_H
#define GRID_MAPPER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cslibs_mapping/mapper/occupancy_grid_mapper_2d.h>
#include <cslibs_mapping/mapper/ndt_grid_mapper_2d.h>

#include <cslibs_math_ros/tf/tf_listener_2d.hpp>

namespace cslibs_mapping {
class MapperNode2d
{
public:
    MapperNode2d();

    bool setup();
    void run();

private:
    using interval_t    = std::array<float, 2>;
    using point_t       = cslibs_math_2d::Point2d;
    using transform_t   = cslibs_math_2d::Transform2d;
    using measurement_t = Measurement<point_t, transform_t>;


    ros::NodeHandle                         nh_;
    std::vector<ros::Subscriber>            sub_lasers_;
    cslibs_math_ros::tf::TFListener2d::Ptr  tf_;
    OccupancyGridMapper2d::Ptr              occ_mapper_;
    NDTGridMapper2d::Ptr                    ndt_mapper_;
    std::string                             map_frame_;

    ros::Publisher                          pub_occ_map_;
    ros::Duration                           pub_occ_interval_;
    ros::Time                               pub_occ_last_time_;

    ros::Publisher                          pub_ndt_map_;
    ros::Duration                           pub_ndt_interval_;
    ros::Time                               pub_ndt_last_time_;

    double                                  node_rate_;

    bool                                    undistortion_;              /// check if undistortion shall be applied
    std::string                             undistortion_fixed_frame_;  /// the fixed frame necessary for the undistortion
    ros::Duration                           tf_timeout_;                /// time out for the tf listener

    interval_t                              laser_linear_interval_;           /// linear field of view
    interval_t                              laser_angular_interval_;          /// angular field of view


    void laserscan(const sensor_msgs::LaserScanConstPtr &msg);

    void publishNDT(const OccupancyGridMapper2d::static_map_stamped_t &map);

    void publishOcc(const OccupancyGridMapper2d::static_map_stamped_t &map);




};
}

#endif // GRID_MAPPER_NODE_H
