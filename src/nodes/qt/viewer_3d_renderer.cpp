#include "viewer_3d_renderer.h"

#include <Qt3DRenderer/QWindow>
#include <Qt3DRenderer/qmesh.h>
#include <Qt3DRenderer/qtechnique.h>
#include <Qt3DRenderer/qmaterial.h>
#include <Qt3DRenderer/qeffect.h>
#include <Qt3DRenderer/qtexture.h>
#include <Qt3DRenderer/qrenderpass.h>
#include <Qt3DRenderer/qsceneloader.h>
#include <Qt3DRenderer/qrenderaspect.h>
#include <Qt3DRenderer/qframegraph.h>
#include <Qt3DRenderer/qforwardrenderer.h>

#include <Qt3DCore/qcamera.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qcameralens.h>
#include <Qt3DCore/qscaletransform.h>
#include <Qt3DCore/qrotatetransform.h>
#include <Qt3DCore/qlookattransform.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DCore/qaspectengine.h>

#include <Qt3DInput/QInputAspect>

#include "distribution3d_mesh.hpp"

namespace cslibs_mapping {
Viewer3dRenderer::Viewer3dRenderer(QObject *parent) :
    QObject(parent)
{
}

Viewer3dRenderer::~Viewer3dRenderer()
{
}

void Viewer3dRenderer::setup(Qt3D::QWindow *view,
                             Viewer3dModel *model,
                             const float zoom_factor)
{
    /// preparing the engine
    view_  = view;
    model_ = model;
    zoom_factor_ = zoom_factor;
    zoom_factor_inv_ = 1.f / zoom_factor;

    aspect_engine_ = new Qt3D::QAspectEngine(view);
    aspect_input_  = new Qt3D::QInputAspect(aspect_engine_);
    aspect_engine_->registerAspect(new Qt3D::QRenderAspect);
    aspect_engine_->registerAspect(aspect_input_);
    aspect_engine_->initialize();

    QVariantMap data;
    data.insert(QStringLiteral("surface"), QVariant::fromValue(static_cast<QSurface *>(view_)));
    data.insert(QStringLiteral("eventSource"), QVariant::fromValue(view_));
    aspect_engine_->setData(data);

    /// build the scene graph
    root_entity_        = new Qt3D::QEntity;
    camera_entity_      = new Qt3D::QCamera(root_entity_);
    frame_graph_        = new Qt3D::QFrameGraph;
    forward_renderer_   = new Qt3D::QForwardRenderer;

    camera_entity_->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera_entity_->setPosition(QVector3D(0, 0, -20.0f)); ///  this has to be scaled using zoom or so ...
    camera_entity_->setUpVector(QVector3D(0, 1, 0));
    camera_entity_->setViewCenter(QVector3D(0, 0, 0));
    camera_entity_->setObjectName(QStringLiteral("cameraEntity"));
    aspect_input_->setCamera(camera_entity_);

    forward_renderer_->setCamera(camera_entity_);
    forward_renderer_->setClearColor(QColor(Qt::gray));
    frame_graph_->setActiveFrameGraph(forward_renderer_);

    root_entity_->addComponent(frame_graph_);

    aspect_engine_->setRootEntity(root_entity_);
}

void Viewer3dRenderer::aspectRatioUpdate(const float ratio)
{
    camera_entity_->lens()->setPerspectiveProjection(45.0f, ratio, 0.1f, 1000.0f);
}

void Viewer3dRenderer::mouseWheelUpdate(const int delta)
{
    camera_entity_->setPosition(camera_entity_->position() * (delta > 0 ? zoom_factor_ : zoom_factor_inv_));
}

bool once = true;

void Viewer3dRenderer::modelUpdate(const update_list_t &updates)
{
    if(!once)
        return;
    once = false;
    for(const auto id : updates) {
        Qt3D::QEntity* &e = entities_[id];
        if(e) {
            e->setParent(static_cast<Qt3D::QNode*>(nullptr));
            delete e;
        };
        e = Distribution3dMesh::create(model_->get(id), root_entity_);
    }
}

Qt3D::QEntity * Viewer3dRenderer::getRootEntity()
{
    return root_entity_;
}
}
