#ifndef OCCUPANCY_NDT_GRID_MAPPER_2D_H
#define OCCUPANCY_NDT_GRID_MAPPER_2D_H

#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <cslibs_utility/synchronized/synchronized_queue.hpp>

#include <cslibs_mapping/measurement/measurement.hpp>

#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>
#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/utility/inverse_model.hpp>
#include <cslibs_gridmaps/utility/delegate.hpp>
#include <cslibs_ndt_2d/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_time/stamped.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_math_2d/linear/box.hpp>

#include <nav_msgs/Path.h>

namespace cslibs_mapping {
class OccupancyNDTGridMapper2d
{
public:
    using Ptr                       = std::shared_ptr<OccupancyNDTGridMapper2d>;
    using lock_t                    = std::unique_lock<std::mutex>;
    using dynamic_map_t             = cslibs_ndt_2d::dynamic_maps::OccupancyGridmap;
    using static_map_t              = cslibs_gridmaps::static_maps::ProbabilityGridmap;
    using static_map_stamped_t      = cslibs_time::Stamped<static_map_t::Ptr>;
    using callback_t                = delegate<void(const static_map_stamped_t &)>;
    using point_t                   = cslibs_math_2d::Point2d;
    using transform_t               = cslibs_math_2d::Transform2d;
    using measurement_t             = Measurement<point_t, transform_t>;

    OccupancyNDTGridMapper2d(
            const cslibs_gridmaps::utility::InverseModel &inverse_model,
            const double                                  resolution,
            const double                                  sampling_resolution,
            const std::string                            &frame_id);

    virtual ~OccupancyNDTGridMapper2d();

    void insert(const measurement_t &measurement);

    void get(static_map_stamped_t &map);

    void requestMap();
    void setCallback(const callback_t &cb);

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
    dynamic_map_t::Ptr                                  dynamic_map_;
    cslibs_gridmaps::utility::InverseModel              inverse_model_;
    double                                              resolution_;
    double                                              sampling_resolution_;
    std::string                                         frame_id_;

    void loop();
    void mapRequest();
    void process(const measurement_t &points);

public:
    bool saveMap(const std::string    & path,
                 const nav_msgs::Path & poses_path);
};
}

#endif // OCCUPANCY_NDT_GRID_MAPPER_2D_H
