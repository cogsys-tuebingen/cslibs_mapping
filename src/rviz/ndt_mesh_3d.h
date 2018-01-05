#ifndef NDT_CIRCLES_3D_H
#define NDT_CIRCLES_3D_H

#include "ndt_visual_3d.h"

namespace cslibs_mapping {
/*
 * This class should later on allow to load custom meshes to represent NDT distributions.
 */
class NDTMeash3D : public NDTVisual3D
{
public:
    using Ptr = std::shared_ptr<NDTMeash3D>;

    NDTMeash3D(Ogre::SceneManager * scene_manager, Ogre::SceneNode *parent_node);
    virtual ~NDTMeash3D();

    void setScale(const Ogre::Vector3 &scale) override;
    void setFramePosition(const Ogre::Vector3 &pos) override;
    void setFrameOrientation(const Ogre::Quaternion &quaternion) override;
    void setColor(const std::array<float,4> &color) override;
    void setColorScale(const float s) override;

protected:
    std::shared_ptr<rviz::Shape>  shape_;
};
}

#endif // NDT_CIRCLES_3D_H
