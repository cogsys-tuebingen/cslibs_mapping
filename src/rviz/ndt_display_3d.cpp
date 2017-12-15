#include "ndt_display_3d.h"
#include "ndt_visual_3d.h"


#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/frame_manager.h>

#include <cslibs_time/time.hpp>

#include <tf/tf.h>

namespace cslibs_mapping {
NDTDisplay3D::NDTDisplay3D() :
    accumulate_(true)
{
    color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                               "Color to draw the acceleration arrows.",
                                               this, SLOT( updateColorAndAlpha() ));

    alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                               "0 is fully transparent, 1.0 is fully opaque.",
                                               this, SLOT( updateColorAndAlpha() ));
    bool_property_ = new rviz::BoolProperty( "Accumulate", true,
                                             "Accumulate all received updates to one NDT map.",
                                             this, SLOT(updateAccumulation() ));
}

NDTDisplay3D::~NDTDisplay3D()
{
}

void NDTDisplay3D::onInitialize()
{
    MFDClass::onInitialize();
}

void NDTDisplay3D::reset()
{
    MFDClass::reset();
    visuals_.clear();
}

void NDTDisplay3D::updateColorAndAlpha()
{
    const Ogre::ColourValue color = color_property_->getOgreColor();

    color_[0] = alpha_property_->getFloat();
    color_[1] = color.r;
    color_[2] = color.g;
    color_[3] = color.b;
    for(auto &v : visuals_) {
        v.second->setColor(color_);
    }
}

void NDTDisplay3D::updateAccumulation()
{
    accumulate_ = bool_property_->getBool();
    if(!accumulate_) {
        visuals_.clear();
    }
}

void NDTDisplay3D::processMessage(const cslibs_mapping::Distribution3dArray::ConstPtr& msg)
{
    /// get the map frame
    Ogre::Quaternion frame_orientation;
    Ogre::Vector3 frame_position;

    if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                    msg->header.stamp,
                                                    frame_position, frame_orientation )) {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                   msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
        return;
    }

    auto valid = [](const Ogre::Vector3 &position,
            const Ogre::Quaternion &orientation,
            const Ogre::Vector3 &scale)
    {
        const bool p = std::isnormal(position.x) && std::isnormal(position.y) && std::isnormal(position.z);
        const bool o = std::isnormal(orientation.x) && std::isnormal(orientation.y) && std::isnormal(orientation.z) && std::isnormal(orientation.w);
        const bool s = std::isnormal(scale.x) && std::isnormal(scale.y) && std::isnormal(scale.z);
        return p && o && s;
    };

    auto getRotation = [&frame_orientation](const Distribution3d &d)
    {
        const tf::Matrix3x3 m(d.eigen_vectors[0].data, d.eigen_vectors[1].data, d.eigen_vectors[2].data,
                d.eigen_vectors[3].data, d.eigen_vectors[4].data, d.eigen_vectors[5].data,
                d.eigen_vectors[6].data, d.eigen_vectors[7].data, d.eigen_vectors[8].data);
        tf::Quaternion q;
        m.getRotation(q);
        return frame_orientation * Ogre::Quaternion(static_cast<float>(q.w()),
                                                    static_cast<float>(q.x()),
                                                    static_cast<float>(q.y()),
                                                    static_cast<float>(q.z()));
    };
    auto getTranslation = [&frame_orientation, &frame_position](const Distribution3d &d)
    {
        return frame_orientation * Ogre::Vector3(static_cast<float>(d.mean[0].data),
                static_cast<float>(d.mean[1].data),
                static_cast<float>(d.mean[2].data)) + frame_position;
    };

    auto getScale = [](const Distribution3d &d)
    {
        return Ogre::Vector3(static_cast<float>(d.eigen_values[0].data),
                static_cast<float>(d.eigen_values[1].data),
                static_cast<float>(d.eigen_values[2].data));
    };

    if(!accumulate_)
        visuals_.clear();

    cslibs_time::Time start = cslibs_time::Time::now();
    for(const auto &d : msg->data) {
        const Ogre::Vector3 p    = getTranslation(d);
        const Ogre::Quaternion q = getRotation(d);
        const Ogre::Vector3 s    = getScale(d);

        if(valid(p,q,s) && std::isnormal(d.prob.data)) {
            NDTVisual3D::Ptr &v = visuals_[d.id.data];
            if(!v) {
                v.reset(new NDTVisual3D( context_->getSceneManager(), scene_node_));
            }
            v->setFramePosition(p);
            v->setFrameOrientation(q);
            v->setScale(s);
            v->setColorScale(static_cast<float>(1.0 - d.prob.data));
            v->setColor(color_);
        }
    }
    std::cerr << (cslibs_time::Time::now() - start).milliseconds() << "ms" << std::endl;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cslibs_mapping::NDTDisplay3D,rviz::Display )
