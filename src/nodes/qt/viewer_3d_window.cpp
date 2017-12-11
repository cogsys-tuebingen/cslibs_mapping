#include "viewer_3d_window.h"

#include <Qt3DRenderer/QWindow>
#include <iostream>
#include <QMouseEvent>

namespace cslibs_mapping {

Viewer3dWindow::Viewer3dWindow(QObject *parent) :
    QObject (parent)
{
}

Viewer3dWindow::~Viewer3dWindow()
{
}

void Viewer3dWindow::setup(const std::string &window_name)
{
    view_ = new Qt3D::QWindow;
    view_container_ = QWidget::createWindowContainer(view_);
    ui_ = new Ui::Viewer3dMainWidget;
    ui_->setupUi(view_container_);

    view_container_->setWindowTitle(window_name.c_str());
    view_container_->show();

    view_container_->installEventFilter(this);
    view_->installEventFilter(this);
}

Ui::Viewer3dMainWidget* Viewer3dWindow::getUi()
{
    return ui_;
}

QWidget * Viewer3dWindow::getViewContainer()
{
    return view_container_;
}

Qt3D::QWindow* Viewer3dWindow::getView()
{
    return view_;
}

float Viewer3dWindow::getAspectRatio()
{
    QSize widgetSize =  view_container_->size();
    float aspectRatio = float(widgetSize.width()) / float(widgetSize.height());
    return aspectRatio;
}

bool Viewer3dWindow::eventFilter(QObject *object, QEvent *event)
{
    if(object == view_container_ && event->type() == QEvent::Resize) {
        aspectRatioChanged();
    }
    if(object == view_ && event->type() == QEvent::Wheel) {
        QWheelEvent * wheel_event = static_cast<QWheelEvent*>(event);
        mouseWheelChanged(wheel_event->delta());
    }
    return QObject::eventFilter(object, event);
}
}
