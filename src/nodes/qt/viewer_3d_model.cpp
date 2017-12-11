#include "viewer_3d_model.h"


namespace cslibs_mapping {
Viewer3dModel::Viewer3dModel() :
    ros_shutdown_(false),
    nh_("~")
{
}

Viewer3dModel::~Viewer3dModel()
{
    ros_shutdown_ = true;
    if(ros_thread_.joinable()) {
        ros_thread_.join();
    }
}

void Viewer3dModel::setup()
{
    const std::string  ndt_topic      = nh_.param<std::string>("ndt_topic", "");
    const unsigned int ndt_queue_size = static_cast<unsigned int>(nh_.param<int>("ndt_queue_size", 1));

    sub_ = nh_.subscribe(ndt_topic, ndt_queue_size, &Viewer3dModel::distribution3d, this);

    ros_thread_ = std::thread([this](){loop();});
}

void Viewer3dModel::distribution3d(const cslibs_mapping::Distributions3d::ConstPtr &msg)
{

}

void Viewer3dModel::loop()
{
    while(!ros_shutdown_ && ros::ok()) {
        ros::spinOnce();
    }

}

}
