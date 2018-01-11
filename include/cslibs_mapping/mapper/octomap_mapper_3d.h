#ifndef OCTOMAP_MAPPER_3D_H
#define OCTOMAP_MAPPER_3D_H

#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <cslibs_utility/synchronized/synchronized_queue.hpp>

#include <cslibs_mapping/measurement/measurement.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>
#include <cslibs_gridmaps/utility/delegate.hpp>
#include <cslibs_ndt_3d/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_time/stamped.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_3d/linear/box.hpp>
#include <cslibs_math/common/array.hpp>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <nav_msgs/Path.h>
#include <set>

namespace cslibs_mapping {
class OctomapMapper3d
{
public:
    using Ptr                       = std::shared_ptr<OctomapMapper3d>;
    using lock_t                    = std::unique_lock<std::mutex>;
    using dynamic_map_t             = octomap::OcTree;
    using static_map_t              = octomap_msgs::Octomap;
    using static_map_stamped_t      = cslibs_time::Stamped<static_map_t::Ptr>;
    using callback_t                = delegate<void(const static_map_stamped_t &)>;
    using point_t                   = cslibs_math_3d::Point3d;
    using transform_t               = cslibs_math_3d::Transform3d;
    using measurement_t             = Measurement<point_t, transform_t>;

public:
    OctomapMapper3d(
            const cslibs_gridmaps::utility::InverseModel &inverse_model,
            const double                                  resolution,
            const std::string                            &frame_id);

    virtual ~OctomapMapper3d();

    void insert(
            const measurement_t & measurement);

    void get(
            static_map_stamped_t & map);

    void requestMap();

    void setCallback(
            const callback_t & cb);

protected:
    cslibs_utility::synchronized::queue<measurement_t>  q_;

    std::thread                                         thread_;
    std::condition_variable                             notify_event_;
    std::mutex                                          notify_event_mutex_;
    std::atomic_bool                                    stop_;
    std::atomic_bool                                    request_map_;
    std::condition_variable                             notify_static_map_;
    std::mutex                                          static_map_mutex_;

    static_map_stamped_t                                static_map_;
    callback_t                                          callback_;

    cslibs_time::Time                                   latest_time_;
    std::shared_ptr<dynamic_map_t>                      dynamic_map_;

    cslibs_gridmaps::utility::InverseModel              inverse_model_;
    double                                              resolution_;
    std::string                                         frame_id_;

    void loop();

    void mapRequest();

    void process(
            const measurement_t & points);

public:
    bool saveMap(
        const std::string    & path,
        const nav_msgs::Path & poses_path);
};
}

#endif // OCTOMAP_MAPPER_3D_H
