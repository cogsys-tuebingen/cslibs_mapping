#ifndef CSLIBS_MAPPING_VIEWER_3D_MODEL_H
#define CSLIBS_MAPPING_VIEWER_3D_MODEL_H

#include <QObject>

#include <ros/ros.h>
#include <cslibs_mapping/Distributions3d.h>
#include <cslibs_utility/synchronized/wrap_around.hpp>

#include <thread>
#include <memory>
#include <atomic>

#include <QVector3D>


namespace cslibs_mapping {
class Viewer3dModel : public QObject
{
    Q_OBJECT
public:
    struct Distribution {
        using synchonized_t = cslibs_utility::synchronized::WrapAround<const Distribution>;

        using Ptr = std::shared_ptr<Distribution>;
        std::mutex  mutex_;

        QVector3D   origin;
        QVector3D   rotation_axis;
        float       rotation_angle;
        QVector3D   eigen_values;
    };

    using mutex_t = std::mutex;
    using lock_t = std::unique_lock<mutex_t>;

    Viewer3dModel();
    virtual ~Viewer3dModel();

    void setup();

    Distribution::synchonized_t get(const unsigned int id);

private:
    std::thread         ros_thread_;
    std::atomic_bool    ros_shutdown_;

    ros::NodeHandle     nh_;
    ros::Subscriber     sub_;

    mutex_t                                   data_mutex_;
    std::map<unsigned int, Distribution::Ptr> data_;

    void loop();
    void distribution3d(const cslibs_mapping::Distributions3d::ConstPtr &msg);



};
}

#endif // CSLIBS_MAPPING_VIEWER_3D_MODEL_H
