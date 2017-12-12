#ifndef CSLIBS_MAPPING_VIEWER_3D_RENDERER_H
#define CSLIBS_MAPPING_VIEWER_3D_RENDERER_H

#include <QObject>
#include <vector>
#include <memory>

namespace Qt3D {
    class QWindow;
    class QEntity;
    class QCamera;
    class QFrameGraph;
    class QForwardRenderer;
    class QInputAspect;
    class QAspectEngine;
}

namespace cslibs_mapping {
class Viewer3dModel;

class Viewer3dRenderer : public QObject
{
    Q_OBJECT

public:
    using update_list_t = std::vector<uint64_t>;
    using update_list_ptr_t = std::shared_ptr<update_list_t>;

    Viewer3dRenderer(QObject *parent = nullptr);
    virtual ~Viewer3dRenderer();

    void setup(Qt3D::QWindow *view,
               Viewer3dModel *model,
               const float zoom_factor = 2.0);
    Qt3D::QEntity *getRootEntity();

public slots:
    void aspectRatioUpdate(const float ratio);
    void mouseWheelUpdate(const int delta);
    void modelUpdate(const update_list_t &updates);


private:
    Qt3D::QWindow                     *view_;
    Viewer3dModel                     *model_;
    std::map<uint64_t, Qt3D::QEntity*> entities_;

    float                   zoom_factor_;
    float                   zoom_factor_inv_;
    Qt3D::QEntity          *root_entity_;
    Qt3D::QCamera          *camera_entity_;
    Qt3D::QFrameGraph      *frame_graph_;
    Qt3D::QForwardRenderer *forward_renderer_;
    Qt3D::QInputAspect     *aspect_input_;
    Qt3D::QAspectEngine    *aspect_engine_;
};
}

#endif // CSLIBS_MAPPING_VIEWER_3D_RENDERER_H
