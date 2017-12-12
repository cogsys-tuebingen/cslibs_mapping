#include "viewer_3d_model.h"
#include <tf/tf.h>

namespace cslibs_mapping {
Viewer3dModel::Viewer3dModel(QObject *parent) :
    QObject(parent),
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
    const std::string  ndt_topic      = nh_.param<std::string>("ndt_topic", "/map/ndt_3d_distributions");
    const unsigned int ndt_queue_size = static_cast<unsigned int>(nh_.param<int>("ndt_queue_size", 1));

    sub_ = nh_.subscribe(ndt_topic, ndt_queue_size, &Viewer3dModel::distribution3d, this);

    ros_thread_ = std::thread([this](){loop();});
}

Viewer3dModel::Distribution::synchonized_t Viewer3dModel::get(const uint64_t id)
{
    lock_t l(data_mutex_);
    const Distribution::Ptr d = data_[id];
    return d ? Distribution::synchonized_t(d.get(), &(d->mutex_)) : Distribution::synchonized_t();
}

void Viewer3dModel::distribution3d(const cslibs_mapping::Distributions3d::ConstPtr &msg)
{
    auto convert = [](const cslibs_mapping::Distribution3d &d)
    {
        Distribution::Ptr distribution(new Distribution);
        distribution->id = d.id.data;
        distribution->eigen_values = QVector3D(static_cast<float>(d.eigen_values[0].data),
                static_cast<float>(d.eigen_values[1].data),
                static_cast<float>(d.eigen_values[2].data));
        distribution->mean = QVector3D(static_cast<float>(d.mean[0].data),
                static_cast<float>(d.mean[1].data),
                static_cast<float>(d.mean[2].data));


        tf::Matrix3x3 rotation(d.eigen_vectors[0].data, d.eigen_vectors[1].data, d.eigen_vectors[2].data,
                               d.eigen_vectors[3].data, d.eigen_vectors[4].data, d.eigen_vectors[5].data,
                               d.eigen_vectors[6].data, d.eigen_vectors[7].data, d.eigen_vectors[8].data);
        tf::Quaternion quaternion;
        rotation.getRotation(quaternion);
        tf::Vector3 quaternion_axis = quaternion.getAxis();
        double quaternion_angle = quaternion.getAngle();
        distribution->rotation_angle = static_cast<float>(quaternion_angle);
        distribution->rotation_axis = QVector3D(static_cast<float>(quaternion_axis[0]),
                                               static_cast<float>(quaternion_axis[1]),
                                               static_cast<float>(quaternion_axis[2]));
        return distribution;
    };

    update_list_t update;
    update.reserve(msg->data.size());
    for(const auto &d : msg->data) {
        auto distribution = convert(d);
        lock_t l(data_mutex_);
        data_[d.id.data] = std::move(distribution);
        update.emplace_back(d.id.data);
    }
    modelChanged(update);
}

void Viewer3dModel::loop()
{
    while(!ros_shutdown_ && ros::ok()) {
        ros::Rate(30.0).sleep();
        ros::spinOnce();
    }

}

}
