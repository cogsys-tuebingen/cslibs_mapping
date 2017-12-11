#ifndef CSLIBS_MAPPING_VIEWER_3D_RENDERER_H
#define CSLIBS_MAPPING_VIEWER_3D_RENDERER_H

#include <QObject>

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
class Viewer3dRenderer : public QObject
{
    Q_OBJECT

public:
    Viewer3dRenderer(QObject *parent = nullptr);
    virtual ~Viewer3dRenderer();

    void setup(Qt3D::QWindow *view,
               const float zoom_factor = 2.0);
    Qt3D::QEntity *getRootEntity();

public slots:
    void aspectRatioUpdate(const float ratio);
    void mouseWheelUpdate(const int delta);


private:
    Qt3D::QWindow          *view_;

    float                  zoom_factor_;
    float                  zoom_factor_inv_;
    Qt3D::QEntity          *root_entity_;
    Qt3D::QCamera          *camera_entity_;
    Qt3D::QFrameGraph      *frame_graph_;
    Qt3D::QForwardRenderer *forward_renderer_;
    Qt3D::QInputAspect     *aspect_input_;
    Qt3D::QAspectEngine    *aspect_engine_;
};
}

#endif // CSLIBS_MAPPING_VIEWER_3D_RENDERER_H
