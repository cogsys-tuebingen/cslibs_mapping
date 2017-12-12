#ifndef CSLIBS_MAPPING_VIEWER_3D_CONTROLLER_H
#define CSLIBS_MAPPING_VIEWER_3D_CONTROLLER_H

#include <QObject>
#include <memory>
#include <vector>

namespace cslibs_mapping {
class Viewer3dWindow;
class Viewer3dModel;
class Viewer3dRenderer;

class Viewer3dController : public QObject
{
    Q_OBJECT

public:
    using update_list_t = std::vector<uint64_t>;
    using update_list_ptr_t = std::shared_ptr<update_list_t>;


    Viewer3dController();
    void setup(Viewer3dWindow   *gui,
               Viewer3dModel    *model,
               Viewer3dRenderer *renderer);

public slots:
    void aspectRatioUpdate();
    void mouseWheelUpdate(const int delta);
    void modelUpdate(const update_list_t &updates);

signals:
    void aspectRatioChanged(const float ratio);
    void mouseWheelChanged(const int delta);
    void modelChanged(const update_list_t &updates);

private:
    Viewer3dWindow   *gui_;
    Viewer3dModel    *model_;
    Viewer3dRenderer *renderer_;

};
}

#endif // CSLIBS_MAPPING_VIEWER_3D_CONTROLLER_H
