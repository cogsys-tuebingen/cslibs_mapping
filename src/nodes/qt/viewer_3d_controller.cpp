#include "viewer_3d_controller.h"

#include "viewer_3d_renderer.h"
#include "viewer_3d_window.h"
#include "viewer_3d_model.h"

#include <QMetaType>

namespace cslibs_mapping {
Viewer3dController::Viewer3dController()
{
}

void Viewer3dController::setup(Viewer3dWindow   *gui,
                               Viewer3dModel    *model,
                               Viewer3dRenderer *renderer)
{
    gui_ = gui;
    model_ = model;
    renderer_ = renderer;

    qRegisterMetaType<update_list_ptr_t>("update list");

    QObject::connect(gui_, &Viewer3dWindow::aspectRatioChanged,
                     this, &Viewer3dController::aspectRatioUpdate,
                     Qt::QueuedConnection);
    QObject::connect(this, &Viewer3dController::aspectRatioChanged,
                     renderer_, &Viewer3dRenderer::aspectRatioUpdate,
                     Qt::QueuedConnection);
    QObject::connect(gui_, &Viewer3dWindow::mouseWheelChanged,
                     this, &Viewer3dController::mouseWheelUpdate,
                     Qt::QueuedConnection);
    QObject::connect(this, &Viewer3dController::mouseWheelChanged,
                     renderer_, &Viewer3dRenderer::mouseWheelUpdate,
                     Qt::QueuedConnection);
    QObject::connect(model_, &Viewer3dModel::modelChanged,
                     this, &Viewer3dController::modelUpdate,
                     Qt::QueuedConnection);
    QObject::connect(this, &Viewer3dController::modelChanged,
                     renderer_, &Viewer3dRenderer::modelUpdate,
                     Qt::QueuedConnection);

    renderer->aspectRatioUpdate(gui_->getAspectRatio());
}

void Viewer3dController::modelUpdate(const update_list_t &updates)
{
    modelChanged(updates);
}

void Viewer3dController::aspectRatioUpdate()
{
    aspectRatioChanged(gui_->getAspectRatio());
}

void Viewer3dController::mouseWheelUpdate(const int delta)
{
    mouseWheelChanged(delta);
}

}
