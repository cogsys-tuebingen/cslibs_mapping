#ifndef CSLIBS_MAPPING_NDT_VISUAL_3D_H
#define CSLIBS_MAPPING_NDT_VISUAL_3D_H

namespace Ogre {
class Vector3;
class Quaternion;
class SceneManager;
class SceneNode;
}

#include <memory>

namespace rviz {
class Shape;
}

namespace cslibs_mapping {
class NDTVisual3D
{
public:
    using Ptr = std::shared_ptr<NDTVisual3D>;

    NDTVisual3D(Ogre::SceneManager * scene_manager, Ogre::SceneNode *parent_node);
    virtual ~NDTVisual3D();

    void setScale(const Ogre::Vector3 &scale);
    void setFramePosition(const Ogre::Vector3 &pos);
    void setFrameOrientation(const Ogre::Quaternion &quaternion);
    void setColor(const std::array<float,4> &color);
    void setColorScale(const float s);

private:
    float                         color_scale_;
    Ogre::SceneManager           *scene_manager_;
    Ogre::SceneNode              *frame_node_;
    std::shared_ptr<rviz::Shape>  shape_;

};
}

#endif // CSLIBS_MAPPING_NDT_VISUAL_3D_H
