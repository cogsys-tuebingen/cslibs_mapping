#ifndef CSLIBS_MAPPING_VIEWER_3D_MODEL_H
#define CSLIBS_MAPPING_VIEWER_3D_MODEL_H

#include <QObject>

#include <ros/ros.h>
#include <cslibs_mapping/Distributions3d.h>

#include <thread>
#include <atomic>


namespace cslibs_mapping {
class Viewer3dModel : public QObject
{
    Q_OBJECT
public:
    Viewer3dModel();
    virtual ~Viewer3dModel();

    void setup();

private:
    std::thread         ros_thread_;
    std::atomic_bool    ros_shutdown_;

    ros::NodeHandle     nh_;
    ros::Subscriber     sub_;

    void loop();
    void distribution3d(const cslibs_mapping::Distributions3d::ConstPtr &msg);

};
}

#endif // CSLIBS_MAPPING_VIEWER_3D_MODEL_H
