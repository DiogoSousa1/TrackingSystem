/**
 * @file GeometryHelpers.h
 * @author Diogo Sousa
 * @brief Helpers for geometry stuff like matrix operations and euler angles conversion
 * @version 1.0
 * @date 2021-08-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef GEOMETRYHELPER_H
#define GEOMETRYHELPER_H
#include <librealsense2/rsutil.h>
#include "TagStructures.h"
#include <math.h>

static EulerAngles convertMatrixToEuler(Matrix matrixToEuler)
{
    EulerAngles euler = {0};
    float sy = sqrt(matrixToEuler.m11 * matrixToEuler.m11 + matrixToEuler.m21 * matrixToEuler.m21);
    bool singular = sy < 1e-6;
    float x, y, z;
    if (!singular)
    {
        x = atan2(matrixToEuler.m32, matrixToEuler.m33);
        y = atan2(-matrixToEuler.m31, sy);
        z = atan2(matrixToEuler.m21, matrixToEuler.m11);
    }
    else
    {
        x = atan2(-matrixToEuler.m22, matrixToEuler.m21);
        y = atan2(-matrixToEuler.m31, sy);
        z = 0;
    }
    euler.x = x;
    euler.y = y;
    euler.z = z;
    return euler;
}

static poseData transformToRS2Structure(const double rotation[9], const double translation[3])
{
    poseData result;

    result.rotation.m11 = static_cast<float>(rotation[0]);
    result.rotation.m12 = static_cast<float>(rotation[1]);
    result.rotation.m13 = static_cast<float>(rotation[2]);
    result.rotation.m21 = static_cast<float>(rotation[3]);
    result.rotation.m22 = static_cast<float>(rotation[4]);
    result.rotation.m23 = static_cast<float>(rotation[5]);
    result.rotation.m31 = static_cast<float>(rotation[6]);
    result.rotation.m32 = static_cast<float>(rotation[7]);
    result.rotation.m33 = static_cast<float>(rotation[8]);

    result.translation.x = static_cast<float>(translation[0]);
    result.translation.y = static_cast<float>(translation[1]);
    result.translation.z = static_cast<float>(translation[2]);

    return result;
}

static poseData transformToPosestructure(const rs2_quaternion &quaternion, const rs2_vector &translation)
{
    poseData tf;
    tf.rotation.m11 = quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z;
    tf.rotation.m12 = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
    tf.rotation.m13 = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
    tf.rotation.m21 = 2 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
    tf.rotation.m22 = quaternion.w * quaternion.w - quaternion.x * quaternion.x + quaternion.y * quaternion.y - quaternion.z * quaternion.z;
    tf.rotation.m23 = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
    tf.rotation.m31 = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
    tf.rotation.m32 = 2 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x);
    tf.rotation.m33 = quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z;
    tf.translation.x = translation.x;
    tf.translation.y = translation.y;
    tf.translation.z = translation.z;
    return tf;
}

#endif