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
#include "TrackingStructures.h"
#include <math.h>

static EulerAngles convertMatrixToEuler(Matrix3 matrixToEuler)
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
    euler.tilt = x;
    euler.pan = y;
    euler.roll = z;
    return euler;
}

static PoseData transformToPoseStructure(const double rotation[9], const double translation[3])
{
    PoseData result;

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

    result.eulerOfRotation = convertMatrixToEuler(result.rotation);

    return result;
}

static PoseData transformToPosestructure(const rs2_quaternion &quaternion, const rs2_vector &translation)
{
    PoseData tf;
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

//Vector3 operators
static Vector3 operator+(Vector3 left, Vector3 right)
{
    Vector3 result;
    result.x = left.x + right.x;
    result.y = left.y + right.y;
    result.z = left.z + right.z;
    return result;
}

static Vector3 operator-(Vector3 left, Vector3 right)
{
    Vector3 result;
    result.x = left.x - right.x;
    result.y = left.y - right.y;
    result.z = left.z - right.z;
    return result;
}

//Matrix operators
static Matrix3 multiplyMatrices(Matrix3 left, Matrix3 right)
{
    Matrix3 result;
    result.m11 = (left.m11 * right.m11) + (left.m12 * right.m21) + (left.m13 * right.m31);
    result.m12 = (left.m11 * right.m12) + (left.m12 * right.m22) + (left.m13 * right.m32);
    result.m13 = (left.m11 * right.m13) + (left.m12 * right.m23) + (left.m13 * right.m33);
    result.m21 = (left.m21 * right.m11) + (left.m22 * right.m21) + (left.m23 * right.m31);
    result.m22 = (left.m21 * right.m12) + (left.m22 * right.m22) + (left.m23 * right.m32);
    result.m23 = (left.m21 * right.m13) + (left.m22 * right.m23) + (left.m23 * right.m33);
    result.m31 = (left.m31 * right.m11) + (left.m32 * right.m21) + (left.m33 * right.m31);
    result.m32 = (left.m31 * right.m12) + (left.m32 * right.m22) + (left.m33 * right.m32);
    result.m33 = (left.m31 * right.m13) + (left.m32 * right.m23) + (left.m33 * right.m33);
    return result;
}

static Matrix3 IdentityMatrix()
{
    Matrix3 identity = {0};
    identity.m11 = 1.0f;
    identity.m22 = 1.0f;
    identity.m33 = 1.0f;
    return identity;
}

static Matrix3 translateMatrix(Matrix3 toTranslate, Vector3 translation)
{
    Matrix3 identity = IdentityMatrix();
    identity.m11 = translation.x;
    identity.m12 = translation.y;
    identity.m13 = translation.z;
    return multiplyMatrices(identity, toTranslate);
}

static Matrix3 operator*(Matrix3 left, Matrix3 right)
{
    return multiplyMatrices(left, right);
}

static Vector3 transform(Vector3 vector, Matrix3 transform)
{
    Vector3 result;
    result.x = (vector.x * transform.m11) + (vector.y * transform.m21) + (vector.z * transform.m31);
    result.y = (vector.x * transform.m12) + (vector.y * transform.m22) + (vector.z * transform.m32);
    result.z = (vector.x * transform.m13) + (vector.y * transform.m23) + (vector.z * transform.m33);
    return result;
}

#endif