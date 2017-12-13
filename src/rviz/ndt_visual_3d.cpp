#include "ndt_visual_3d.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreLodStrategyManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <OgreEntity.h>

namespace cslibs_mapping {
NDTVisual3D::NDTVisual3D(Ogre::SceneManager *scene_manager,
                         Ogre::SceneNode *parent_node) :
    color_scale_(1.f),
    scene_manager_(scene_manager),
    frame_node_(parent_node->createChildSceneNode()),
    shape_(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_))
{


    Ogre::MaterialPtr mat = shape_->getMaterial();
    Ogre::Entity     *ent = shape_->getEntity();

    mat->setShadingMode(Ogre::ShadeOptions::SO_FLAT);
    mat->setTextureAnisotropy(0);
    mat->setTextureFiltering(Ogre::TextureFilterOptions::TFO_NONE);
    ent->setCastShadows(false);


    //    mat->setCullingMode(Ogre::CULL_NONE ); //or CULL_CLOCKWISE or CULL_ANTICLOCKWISE as you wish
//    mat->setSceneBlending(Ogre::SBT_ADD); // to put some pseudo transparency

}

NDTVisual3D::~NDTVisual3D()
{
    scene_manager_->destroySceneNode( frame_node_ );
}

void NDTVisual3D::setScale(const Ogre::Vector3 &scale)
{
    shape_->setScale(scale);

}

void NDTVisual3D::setFramePosition(const Ogre::Vector3 &pos)
{
    shape_->setPosition(pos);
}

void NDTVisual3D::setFrameOrientation(const Ogre::Quaternion &quaternion)
{
    shape_->setOrientation(quaternion);
}

void NDTVisual3D::setColor(const std::array<float, 4> &color)
{
    shape_->setColor(color_scale_ * color[1],
            color_scale_ * color[2],
            color_scale_ * color[3],
            color[0]);
}

void NDTVisual3D::setColorScale(const float s)
{
    color_scale_ = s;
}
}
