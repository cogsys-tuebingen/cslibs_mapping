#ifndef CSLIBS_MAPPING_NDT_DISPLAY_3D_H
#define CSLIBS_MAPPING_NDT_DISPLAY_3D_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>

#include <cslibs_mapping/Distribution3dArray.h>
#endif

#include <map>
#include <unordered_map>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
}


namespace cslibs_mapping {
class NDTVisual3D;

class NDTDisplay3D : public rviz::MessageFilterDisplay<cslibs_mapping::Distribution3dArray>
{
    Q_OBJECT
public:
    NDTDisplay3D();
    virtual ~NDTDisplay3D();

protected:
    virtual void onInitialize();
    virtual void reset();

private Q_SLOTS:
    void updateColorAndAlpha();
    void updateAccumulation();

private:
    void processMessage( const cslibs_mapping::Distribution3dArray::ConstPtr& msg );

    bool accumulate_;
    std::map<uint64_t, std::shared_ptr<NDTVisual3D>> visuals_;

    std::array<float,4> color_;

    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::BoolProperty*  bool_property_;
};
}


#endif // CSLIBS_MAPPING_NDT_DISPLAY_3D_H
