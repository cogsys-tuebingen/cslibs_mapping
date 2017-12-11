#ifndef CSLIBS_MAPPING_DISTRIBUTION3D_MESH_HPP
#define CSLIBS_MAPPING_DISTRIBUTION3D_MESH_HPP

#include <cslibs_mapping/Distribution3d.h>
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
    Qt3D::QEntity * create(const Distribution3d &distribution)
    {

        // Sphere shape data
        Qt3D::QSphereMesh *sphereMesh = new Qt3D::QSphereMesh();
        sphereMesh->setRings(20);
        sphereMesh->setSlices(20);
        sphereMesh->setRadius(1.0);

        // Sphere mesh transform
        Qt3D::QScaleTransform     *sphereScale       = new Qt3D::QScaleTransform;
        Qt3D::QTranslateTransform *sphereTranslation = new Qt3D::QTranslateTransform;
        Qt3D::QTransform          *sphereTransforms  = new Qt3D::QTransform;
        Qt3D::QRotateTransform    *sphereRotation    = new Qt3D::QRotateTransform;


        sphereScale->setScale3D(QVector3D(static_cast<float>(distribution.eigen_values[0].data),
                                static_cast<float>(distribution.eigen_values[1].data),
                static_cast<float>(distribution.eigen_values[2].data)));
        sphereTranslation->setTranslation(QVector3D(static_cast<float>(distribution.mean[0].data),
                                          static_cast<float>(distribution.mean[1].data),
                static_cast<float>(distribution.mean[2].data)));

        tf::Matrix3x3 rotation(distribution.eigen_vectors[0].data, distribution.eigen_vectors[1].data, distribution.eigen_vectors[2].data,
                               distribution.eigen_vectors[3].data, distribution.eigen_vectors[4].data, distribution.eigen_vectors[5].data,
                               distribution.eigen_vectors[6].data, distribution.eigen_vectors[7].data, distribution.eigen_vectors[8].data);
        tf::Quaternion quaternion;
        rotation.getRotation(quaternion);
        tf::Vector3 quaternion_axis = quaternion.getAxis();
        double quaternion_angle = quaternion.getAngle();

        sphereRotation->setAxis(QVector3D(static_cast<float>(quaternion_axis[0]),
                                          static_cast<float>(quaternion_axis[1]),
                                          static_cast<float>(quaternion_axis[2])));
        sphereRotation->setAngleRad(static_cast<float>(quaternion_angle));

        sphereTransforms->addTransform(sphereScale);
        sphereTransforms->addTransform(sphereRotation);
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
