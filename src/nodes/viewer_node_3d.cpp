#include "qt/scenemodifier.h"

#include <ui_cslibs_mapping_viewer_node_3d.h>
#include <ui_widget.h>

#include <QGuiApplication>

#include <Qt3DRenderer/QWindow>
#include <Qt3DCore/qcamera.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qcameralens.h>

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QCommandLinkButton>
#include <QtGui/QScreen>

#include <Qt3DInput/QInputAspect>

#include <Qt3DRenderer/qtorusmesh.h>
#include <Qt3DRenderer/qmesh.h>
#include <Qt3DRenderer/qtechnique.h>
#include <Qt3DRenderer/qmaterial.h>
#include <Qt3DRenderer/qeffect.h>
#include <Qt3DRenderer/qtexture.h>
#include <Qt3DRenderer/qrenderpass.h>
#include <Qt3DRenderer/qsceneloader.h>

#include <Qt3DCore/qscaletransform.h>
#include <Qt3DCore/qrotatetransform.h>
#include <Qt3DCore/qlookattransform.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DCore/qaspectengine.h>

#include <Qt3DRenderer/qrenderaspect.h>
#include <Qt3DRenderer/qframegraph.h>
#include <Qt3DRenderer/qforwardrenderer.h>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    Qt3D::QWindow *view = new Qt3D::QWindow();
    QWidget *container = QWidget::createWindowContainer(view);
    QSize screenSize = view->screen()->size();
    container->setMinimumSize(QSize(200, 100));
    container->setMaximumSize(screenSize);


    QWidget *widget = new QWidget;
    Ui::MainWidget *ui_ = new Ui::MainWidget;
    ui_->setupUi(widget);
    ui_->layout_horizontal->addWidget(container, 1);
    ui_->layout_vertical->setAlignment(Qt::AlignTop);

    widget->setWindowTitle(QStringLiteral("Basic shapes"));

    Qt3D::QAspectEngine engine;
    engine.registerAspect(new Qt3D::QRenderAspect());
    Qt3D::QInputAspect *input = new Qt3D::QInputAspect;
    engine.registerAspect(input);
    engine.initialize();
    QVariantMap data;
    data.insert(QStringLiteral("surface"), QVariant::fromValue(static_cast<QSurface *>(view)));
    data.insert(QStringLiteral("eventSource"), QVariant::fromValue(view));
    engine.setData(data);

    // Root entity
    Qt3D::QEntity *rootEntity = new Qt3D::QEntity();

    // Camera
    Qt3D::QCamera *cameraEntity = new Qt3D::QCamera(rootEntity);
    cameraEntity->setObjectName(QStringLiteral("cameraEntity"));

    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(0, 0, -20.0f));
    cameraEntity->setUpVector(QVector3D(0, 1, 0));
    cameraEntity->setViewCenter(QVector3D(0, 0, 0));
    input->setCamera(cameraEntity);

    // FrameGraph
    Qt3D::QFrameGraph *frameGraph = new Qt3D::QFrameGraph();
    Qt3D::QForwardRenderer *forwardRenderer = new Qt3D::QForwardRenderer();

    forwardRenderer->setCamera(cameraEntity);
    forwardRenderer->setClearColor(QColor(QRgb(0x4d4d4f)));
    frameGraph->setActiveFrameGraph(forwardRenderer);

    // Setting the FrameGraph
    rootEntity->addComponent(frameGraph);

    // Scenemodifier
    SceneModifier *modifier = new SceneModifier(rootEntity);

    // Set root object of the scene
    engine.setRootEntity(rootEntity);

    // Create control widgets
    QCommandLinkButton *info = new QCommandLinkButton();
    info->setText(QStringLiteral("Qt3D ready-made meshes"));
    info->setDescription(QStringLiteral("Qt3D provides several ready-made meshes, like torus, cylinder, cube and sphere."));
    info->setIconSize(QSize(0,0));

    QCheckBox *torusCB = new QCheckBox(widget);
    torusCB->setChecked(true);
    torusCB->setText(QStringLiteral("Torus"));

    QCheckBox *cylinderCB = new QCheckBox(widget);
    cylinderCB->setChecked(true);
    cylinderCB->setText(QStringLiteral("Cylinder"));

    QCheckBox *cuboidCB = new QCheckBox(widget);
    cuboidCB->setChecked(true);
    cuboidCB->setText(QStringLiteral("Cuboid"));

    QCheckBox *sphereCB = new QCheckBox(widget);
    sphereCB->setChecked(true);
    sphereCB->setText(QStringLiteral("Sphere"));

    ui_->layout_vertical->addWidget(info);
    ui_->layout_vertical->addWidget(torusCB);
    ui_->layout_vertical->addWidget(cylinderCB);
    ui_->layout_vertical->addWidget(cuboidCB);
    ui_->layout_vertical->addWidget(sphereCB);

    QObject::connect(torusCB, &QCheckBox::stateChanged,
                     modifier, &SceneModifier::enableTorus);
    QObject::connect(cylinderCB, &QCheckBox::stateChanged,
                     modifier, &SceneModifier::enableCylinder);
    QObject::connect(cuboidCB, &QCheckBox::stateChanged,
                     modifier, &SceneModifier::enableCuboid);
    QObject::connect(sphereCB, &QCheckBox::stateChanged,
                     modifier, &SceneModifier::enableSphere);

    torusCB->setChecked(true);
    cylinderCB->setChecked(true);
    cuboidCB->setChecked(true);
    sphereCB->setChecked(true);

    // Show window
    widget->show();
    widget->resize(1200, 800);

    // Update the aspect ratio
    QSize widgetSize =  container->size();
    float aspectRatio = float(widgetSize.width()) / float(widgetSize.height());
    cameraEntity->lens()->setPerspectiveProjection(45.0f, aspectRatio, 0.1f, 1000.0f);

    return app.exec();
}
