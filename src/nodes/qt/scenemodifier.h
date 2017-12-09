#ifndef SCENEMODIFIER_H
#define SCENEMODIFIER_H

#include <QtCore/QObject>

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qscaletransform.h>
#include <Qt3DCore/qrotatetransform.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DCore/QTranslateTransform>

#include <Qt3DRenderer/QTorusMesh>
#include <Qt3DRenderer/QCylinderMesh>
#include <Qt3DRenderer/QCuboidMesh>
#include <Qt3DRenderer/QSphereMesh>
#include <Qt3DRenderer/QPhongMaterial>

class SceneModifier : public QObject
{
    Q_OBJECT

public:
    explicit SceneModifier(Qt3D::QEntity *rootEntity);
    ~SceneModifier();

public slots:
    void enableTorus(bool enabled);
    void enableCylinder(bool enabled);
    void enableCuboid(bool enabled);
    void enableSphere(bool enabled);

private:
    Qt3D::QEntity *m_rootEntity;
    Qt3D::QTorusMesh *m_torus;
    Qt3D::QEntity *m_cylinderEntity;
    Qt3D::QEntity *m_torusEntity;
    Qt3D::QEntity *m_cuboidEntity;
    Qt3D::QEntity *m_sphereEntity;
};

#endif // SCENEMODIFIER_H
