#include "ndt_mesh_3d.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreLodStrategyManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <OgreEntity.h>

namespace cslibs_mapping {
NDTMeash3D::NDTMeash3D(Ogre::SceneManager *scene_manager,
                         Ogre::SceneNode *parent_node) :
    NDTVisual3D(scene_manager, parent_node),
    shape_(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_))
{



    Ogre::MaterialPtr mat = shape_->getMaterial();
    Ogre::Entity     *ent = shape_->getEntity();

    mat->setShadingMode(Ogre::ShadeOptions::SO_FLAT);
    mat->setTextureAnisotropy(0);
    mat->setTextureFiltering(Ogre::TextureFilterOptions::TFO_NONE);
    mat->setTransparencyCastsShadows(false);
    mat->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PolygonMode::PM_SOLID);
    ent->setCastShadows(false);


}

NDTMeash3D::~NDTMeash3D()
{
}

void NDTMeash3D::setScale(const Ogre::Vector3 &scale)
{
    shape_->setScale(scale);

}

void NDTMeash3D::setFramePosition(const Ogre::Vector3 &pos)
{
    shape_->setPosition(pos);
}

void NDTMeash3D::setFrameOrientation(const Ogre::Quaternion &quaternion)
{
    shape_->setOrientation(quaternion);
}

void NDTMeash3D::setColor(const std::array<float, 4> &color)
{
    shape_->setColor(color_scale_ * color[1],
            color_scale_ * color[2],
            color_scale_ * color[3],
            color[0]);
}

void NDTMeash3D::setColorScale(const float s)
{
    color_scale_ = s;
}
}
