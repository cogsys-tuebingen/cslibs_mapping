#ifndef CSLIBS_MAPPING_DISTRIBUTION3D_MESH_HPP
#define CSLIBS_MAPPING_DISTRIBUTION3D_MESH_HPP

#include "viewer_3d_model.h"

#include <tf/tf.h>

#include <Qt3DRenderer/QSphereMesh>
#include <Qt3DRenderer/QPhongMaterial>
#include <Qt3DCore/QScaleTransform>
#include <Qt3DCore/QTranslateTransform>
#include <Qt3DCore/QTransform>
#include <Qt3DCore/QRotateTransform>
#include <Qt3DCore/QEntity>
#include <QVector3D>


namespace cslibs_mapping {
struct Distribution3dMesh {
    static Qt3D::QEntity* create(const Viewer3dModel::Distribution::synchonized_t &distribution,
                                 Qt3D::QEntity *parent)
    {

        // Sphere shape data
        Qt3D::QSphereMesh *sphereMesh = new Qt3D::QSphereMesh(parent);
        sphereMesh->setRings(20);
        sphereMesh->setSlices(20);
        sphereMesh->setRadius(1.0);

        // Sphere mesh transform
        Qt3D::QScaleTransform     *sphereScale       = new Qt3D::QScaleTransform;
        Qt3D::QTranslateTransform *sphereTranslation = new Qt3D::QTranslateTransform;
        Qt3D::QTransform          *sphereTransforms  = new Qt3D::QTransform;
//        Qt3D::QRotateTransform    *sphereRotation    = new Qt3D::QRotateTransform;


        sphereScale->setScale3D(distribution->eigen_values);
        sphereTranslation->setTranslation(distribution->mean);
//        sphereRotation->setAxis(distribution->rotation_axis);
//        sphereRotation->setAngleRad(distribution->rotation_angle);

        sphereTransforms->addTransform(sphereScale);
//        sphereTransforms->addTransform(sphereRotation);
        sphereTransforms->addTransform(sphereTranslation);

        Qt3D::QPhongMaterial *sphereMaterial = new Qt3D::QPhongMaterial();
        sphereMaterial->setDiffuse(QColor(QRgb(0xa69929)));
        Qt3D::QEntity *sphere_entity = new Qt3D::QEntity;

        sphere_entity->addComponent(sphereMesh);
        sphere_entity->addComponent(sphereMaterial);
        sphere_entity->addComponent(sphereTransforms);

        return sphere_entity;
    }
};
}

#endif // CSLIBS_MAPPING_DISTRIBUTION3D_MESH_HPP
