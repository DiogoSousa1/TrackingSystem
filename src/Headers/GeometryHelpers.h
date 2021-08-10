/**
 * @file GeometryHelpers.h
 * @author Diogo Sousa
 * @brief 
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

static EulerAngles convertMatrixToEuler(float matrixToEuler[9])
{
    EulerAngles euler = {0};
    float sy = sqrt(matrixToEuler[0] * matrixToEuler[0] + matrixToEuler[3] * matrixToEuler[3]);
    bool singular = sy < 1e-6;
    float x, y, z;
    if (!singular)
    {
        x = atan2(matrixToEuler[7], matrixToEuler[8]);
        y = atan2(-matrixToEuler[6], sy);
        z = atan2(matrixToEuler[3], matrixToEuler[0]);
    }
    else
    {
        x = atan2(-matrixToEuler[4], matrixToEuler[3]);
        y = atan2(-matrixToEuler[6], sy);
        z = 0;
    }
    euler.x = x;
    euler.y = y;
    euler.z = z;
    return euler;
}

static rs2_extrinsics transformToRS2Structure(const double rotation[9], const double translation[3])
{
    rs2_extrinsics result;
    for (int r = 0; r < 9; r++)
    {
        result.rotation[r] = static_cast<float>(rotation[r]);
    }
    for (int t = 0; t < 3; t++)
    {
        result.translation[t] = static_cast<float>(translation[t]);
    }
    return result;
}

static rs2_extrinsics transformToRS2Structure(const rs2_quaternion &quaternion, const rs2_vector &translation)
{
    rs2_extrinsics tf;
    tf.rotation[0] = quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z;
    tf.rotation[1] = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
    tf.rotation[2] = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
    tf.rotation[3] = 2 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
    tf.rotation[4] = quaternion.w * quaternion.w - quaternion.x * quaternion.x + quaternion.y * quaternion.y - quaternion.z * quaternion.z;
    tf.rotation[5] = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
    tf.rotation[6] = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
    tf.rotation[7] = 2 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x);
    tf.rotation[8] = quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z;
    tf.translation[0] = translation.x;
    tf.translation[1] = translation.y;
    tf.translation[2] = translation.z;
    return tf;
}

#endif