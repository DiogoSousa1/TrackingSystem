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

typedef rs2_quaternion Quaternion;

static PoseData calculateWorldPosition(PoseData left, PoseData right)
{
    //is using cameraRotation and cameraPosition just because yes
    PoseData result;
    result.cameraRotation.m11 = left.cameraRotation.m11 * right.cameraRotation.m11 + left.cameraRotation.m12 * right.cameraRotation.m21 + left.cameraRotation.m13 * right.cameraRotation.m31;
    result.cameraRotation.m12 = left.cameraRotation.m11 * right.cameraRotation.m12 + left.cameraRotation.m12 * right.cameraRotation.m22 + left.cameraRotation.m13 * right.cameraRotation.m32;
    result.cameraRotation.m13 = left.cameraRotation.m11 * right.cameraRotation.m13 + left.cameraRotation.m12 * right.cameraRotation.m23 + left.cameraRotation.m13 * right.cameraRotation.m33;
    result.cameraRotation.m21 = left.cameraRotation.m21 * right.cameraRotation.m11 + left.cameraRotation.m22 * right.cameraRotation.m21 + left.cameraRotation.m23 * right.cameraRotation.m31;
    result.cameraRotation.m22 = left.cameraRotation.m21 * right.cameraRotation.m12 + left.cameraRotation.m22 * right.cameraRotation.m22 + left.cameraRotation.m23 * right.cameraRotation.m32;
    result.cameraRotation.m23 = left.cameraRotation.m21 * right.cameraRotation.m13 + left.cameraRotation.m22 * right.cameraRotation.m23 + left.cameraRotation.m23 * right.cameraRotation.m33;
    result.cameraRotation.m31 = left.cameraRotation.m31 * right.cameraRotation.m11 + left.cameraRotation.m32 * right.cameraRotation.m21 + left.cameraRotation.m33 * left.cameraRotation.m31;
    result.cameraRotation.m32 = left.cameraRotation.m31 * right.cameraRotation.m12 + left.cameraRotation.m32 * right.cameraRotation.m22 + left.cameraRotation.m33 * right.cameraRotation.m32;
    result.cameraRotation.m33 = left.cameraRotation.m31 * right.cameraRotation.m13 + left.cameraRotation.m32 * right.cameraRotation.m23 + left.cameraRotation.m33 * right.cameraRotation.m33;

    result.cameraPosition.x = left.cameraRotation.m11 * right.cameraPosition.x + left.cameraRotation.m12 * right.cameraPosition.y + left.cameraRotation.m13 * right.cameraPosition.z + left.cameraPosition.x;
    result.cameraPosition.y = left.cameraRotation.m21 * right.cameraPosition.x + left.cameraRotation.m22 * right.cameraPosition.y + left.cameraRotation.m23 * right.cameraPosition.z + left.cameraPosition.y;
    result.cameraPosition.z = left.cameraRotation.m31 * right.cameraPosition.x + left.cameraRotation.m32 * right.cameraPosition.y + left.cameraRotation.m33 * right.cameraPosition.z + left.cameraPosition.z;
    return result;
}

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

static PoseData transformToPoseStructure(const float rotation[9], const float translation[3])
{
    PoseData result;
    result.cameraRotation.m11 = rotation[0];
    result.cameraRotation.m12 = rotation[1];
    result.cameraRotation.m13 = rotation[2];
    result.cameraRotation.m21 = rotation[3];
    result.cameraRotation.m22 = rotation[4];
    result.cameraRotation.m23 = rotation[5];
    result.cameraRotation.m31 = rotation[6];
    result.cameraRotation.m32 = rotation[7];
    result.cameraRotation.m33 = rotation[8];

    result.cameraPosition.x = translation[0];
    result.cameraPosition.y = translation[1];
    result.cameraPosition.z = translation[2];

    result.eulerOfRotation = convertMatrixToEuler(result.cameraRotation);
    return result;
}

static PoseData transformToPoseStructure(const double rotation[9], const double translation[3])
{
    PoseData result;

    result.cameraRotation.m11 = static_cast<float>(rotation[0]);
    result.cameraRotation.m12 = static_cast<float>(rotation[1]);
    result.cameraRotation.m13 = static_cast<float>(rotation[2]);
    result.cameraRotation.m21 = static_cast<float>(rotation[3]);
    result.cameraRotation.m22 = static_cast<float>(rotation[4]);
    result.cameraRotation.m23 = static_cast<float>(rotation[5]);
    result.cameraRotation.m31 = static_cast<float>(rotation[6]);
    result.cameraRotation.m32 = static_cast<float>(rotation[7]);
    result.cameraRotation.m33 = static_cast<float>(rotation[8]);

    result.cameraPosition.x = static_cast<float>(translation[0]);
    result.cameraPosition.y = static_cast<float>(translation[1]);
    result.cameraPosition.z = static_cast<float>(translation[2]);

    result.eulerOfRotation = convertMatrixToEuler(result.cameraRotation);

    return result;
}

static PoseData transformToPosestructure(const rs2_quaternion &quaternion, const rs2_vector &translation)
{
    PoseData tf;
    tf.cameraRotation.m11 = quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z;
    tf.cameraRotation.m12 = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
    tf.cameraRotation.m13 = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
    tf.cameraRotation.m21 = 2 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
    tf.cameraRotation.m22 = quaternion.w * quaternion.w - quaternion.x * quaternion.x + quaternion.y * quaternion.y - quaternion.z * quaternion.z;
    tf.cameraRotation.m23 = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
    tf.cameraRotation.m31 = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
    tf.cameraRotation.m32 = 2 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x);
    tf.cameraRotation.m33 = quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z;
    tf.cameraPosition.x = translation.x;
    tf.cameraPosition.y = translation.y;
    tf.cameraPosition.z = translation.z;
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

static Matrix3 IdentityMatrix()
{
    Matrix3 identity = {0};
    identity.m11 = 1.0f;
    identity.m22 = 1.0f;
    identity.m33 = 1.0f;
    return identity;
}

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

static Matrix3 Invert(Matrix3 value)
{
    Matrix3 result = {0};
    float det = value.m11 * (value.m22 * value.m33 - value.m31 * value.m23) -
                value.m12 * (value.m21 * value.m33 - value.m23 * value.m31) +
                value.m13 * (value.m21 * value.m32 - value.m22 * value.m31);
    if (det == 0)
    {
        return result;
    }
    float inverseDet = 1 / det;
    result.m11 = (value.m22 * value.m33 - value.m32 * value.m23) * inverseDet;
    result.m12 = (value.m13 * value.m32 - value.m12 * value.m33) * inverseDet;
    result.m13 = (value.m12 * value.m13 - value.m13 * value.m22) * inverseDet;
    result.m21 = (value.m23 * value.m31 - value.m21 * value.m33) * inverseDet;
    result.m22 = (value.m11 * value.m33 - value.m13 * value.m31) * inverseDet;
    result.m23 = (value.m21 * value.m13 - value.m11 * value.m23) * inverseDet;
    result.m31 = (value.m21 * value.m32 - value.m31 * value.m22) * inverseDet;
    result.m32 = (value.m31 * value.m12 - value.m11 * value.m32) * inverseDet;
    result.m33 = (value.m11 * value.m22 - value.m21 * value.m12) * inverseDet;
    return result;
}

static Matrix3 transpose(Matrix3 value)
{
    Matrix3 result = {0};
    result.m11 = value.m11;
    result.m12 = value.m21;
    result.m13 = value.m31;
    result.m21 = value.m12;
    result.m22 = value.m22;
    result.m23 = value.m32;
    result.m31 = value.m13;
    result.m32 = value.m23;
    result.m33 = value.m33;
    return result;
}

static Matrix3 rotateX(float angle)
{
    Matrix3 result = IdentityMatrix();
    float cosVal = cos(angle);
    float sinVal = sin(angle);
    result.m22 = cosVal;
    result.m23 = sinVal;
    result.m32 = -sinVal;
    result.m33 = cosVal;
    return result;
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

static Matrix3 quaternionToMatrix(Quaternion q)
{
    Matrix3 result = {0};
    float sqw = q.w * q.w;
    float sqx = q.x * q.x;
    float sqy = q.y * q.y;
    float sqz = q.z * q.z;
    float inverse = 1 / (sqx + sqy + sqz + sqw);
    result.m11 = (sqx - sqy - sqz + sqw) * inverse;
    result.m22 = (-sqx + sqy - sqz + sqw) * inverse;
    result.m33 = (-sqx - sqy + sqz + sqw) * inverse;

    float tmp1 = q.x * q.y;
    float tmp2 = q.z * q.w;
    result.m21 = 2.0f * (tmp1 + tmp2) * inverse;
    result.m12 = 2.0f * (tmp1 - tmp2) * inverse;

    tmp1 = q.x * q.z;
    tmp2 = q.y * q.w;
    result.m31 = 2.0f * (tmp1 - tmp2) * inverse;
    result.m13 = 2.0f * (tmp1 + tmp2) * inverse;

    tmp1 = q.y * q.z;
    tmp2 = q.x * q.w;
    result.m32 = 2.0f * (tmp1 + tmp2) * inverse;
    result.m23 = 2.0f * (tmp1 - tmp2) * inverse;

    return result;
}

#endif