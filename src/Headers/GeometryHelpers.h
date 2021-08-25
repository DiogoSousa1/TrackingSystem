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

const double RadiansInDegrees = 57.295779513;

//Vector3 operators------------------------------------------------------

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

//Matrix operators------------------------------

/**
 * @brief creates an identity matrix
 * 
 * @return Matrix3 
 */
static Matrix3 IdentityMatrix()
{
    Matrix3 identity = {0};
    identity.m11 = 1.0f;
    identity.m22 = 1.0f;
    identity.m33 = 1.0f;
    return identity;
}

/**
 * @brief Multiply matrices using left(rows) * right(columns) convention
 * 
 * @param left 
 * @param right 
 * @return Matrix3 
 */
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

/**
 * @brief Inverts a matrix3
 * 
 * @param value 
 * @return Matrix3 
 */
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

/**
 * @brief Transpose a matrix3
 * 
 * @param value 
 * @return Matrix3 
 */
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
/**
 * @brief Rotate around x axis clockwise
 * 
 * @param angle 
 * @return Matrix3 
 */
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
/**
 * @brief Rotate around z axis clockwise
 * 
 * @param angle 
 * @return Matrix3 
 */
static Matrix3 rotateZ(float angle)
{
    Matrix3 result = IdentityMatrix();
    float cosVal = cos(angle);
    float sinVal = sin(angle);
    result.m11 = cosVal;
    result.m12 = sinVal;
    result.m21 = -sinVal;
    result.m22 = cosVal;
    return result;
}

/**
 * @brief Rotate around y axis clockwise
 * 
 * @param angle 
 * @return Matrix3 
 */
static Matrix3 rotateY(float angle)
{
    Matrix3 result = IdentityMatrix();
    float cosVal = cos(angle);
    float sinVal = sin(angle);
    result.m11 = cosVal;
    result.m13 = -sinVal;
    result.m31 = sinVal;
    result.m33 = cosVal;
    return result;
}

static Matrix3 convertArrayToMatrix3(const float vector[9], bool isColumnMajor)
{
    Matrix3 result = {0};
    if (isColumnMajor)
    {
        result.m11 = vector[0];
        result.m21 = vector[1];
        result.m31 = vector[2];
        result.m12 = vector[3];
        result.m22 = vector[4];
        result.m32 = vector[5];
        result.m13 = vector[6];
        result.m23 = vector[7];
        result.m33 = vector[8];
    }
    else
    {

        result.m11 = vector[0];
        result.m12 = vector[1];
        result.m13 = vector[2];
        result.m21 = vector[3];
        result.m22 = vector[4];
        result.m23 = vector[5];
        result.m31 = vector[6];
        result.m32 = vector[7];
        result.m33 = vector[8];
    }

    return result;
}

static Matrix3 convertArrayToMatrix3(const double vector[9], bool isColumnMajor)
{
    Matrix3 result = {0};
    if (isColumnMajor)
    {
        result.m11 = static_cast<float>(vector[0]);
        result.m21 = static_cast<float>(vector[1]);
        result.m31 = static_cast<float>(vector[2]);
        result.m12 = static_cast<float>(vector[3]);
        result.m22 = static_cast<float>(vector[4]);
        result.m32 = static_cast<float>(vector[5]);
        result.m13 = static_cast<float>(vector[6]);
        result.m23 = static_cast<float>(vector[7]);
        result.m33 = static_cast<float>(vector[8]);
    }
    else
    {

        result.m11 = static_cast<float>(vector[0]);
        result.m12 = static_cast<float>(vector[1]);
        result.m13 = static_cast<float>(vector[2]);
        result.m21 = static_cast<float>(vector[3]);
        result.m22 = static_cast<float>(vector[4]);
        result.m23 = static_cast<float>(vector[5]);
        result.m31 = static_cast<float>(vector[6]);
        result.m32 = static_cast<float>(vector[7]);
        result.m33 = static_cast<float>(vector[8]);
    }
    return result;
}

/**
 * @brief Multiplies two matrices using convention right applied first
 * 
 * @param left 
 * @param right 
 * @return Matrix3 
 */
static Matrix3 operator*(Matrix3 left, Matrix3 right)
{
    return multiplyMatrices(right, left);
}
/**
 * @brief Transforms quaternion to matrix
 * 
 * @param q rotationMatrix
 * @return Matrix3 
 */
static Matrix3 quaternionToMatrix(Quaternion q)
{
    Matrix3 result = {0};
    float sqw = q.w * q.w;
    float sqx = q.x * q.x;
    float sqy = q.y * q.y;
    float sqz = q.z * q.z;

    float invs = 1.0f / (sqx + sqy + sqz + sqw);

    result.m11 = (sqx - sqy - sqz + sqw) * invs;
    result.m22 = (-sqx + sqy - sqz + sqw) * invs;
    result.m33 = (-sqx - sqy + sqz + sqw) * invs;

    float tmp1 = q.x * q.y;
    float tmp2 = q.z * q.w;

    result.m21 = 2.0f * (tmp1 + tmp2) * invs;
    result.m12 = 2.0f * (tmp1 - tmp2) * invs;

    tmp1 = q.x * q.z;
    tmp2 = q.y * q.w;
    result.m31 = 2.0f * (tmp1 - tmp2) * invs;
    result.m13 = 2.0f * (tmp1 + tmp2) * invs;

    tmp1 = q.y * q.z;
    tmp2 = q.x * q.w;
    result.m32 = 2.0f * (tmp1 + tmp2) * invs;
    result.m23 = 2.0f * (tmp1 - tmp2) * invs;

    return result;
}

/**
 * @brief transform vector point to new coordinate system
 * 
 * @param vector 
 * @param transform 
 * @return Vector3 
 */
static Vector3 transformCoordinate(Vector3 vector, Matrix3 transform)
{
    Vector3 result = {0};
    result.x = vector.x * transform.m11 + vector.y * transform.m21 + vector.z * transform.m31;
    result.y = vector.x * transform.m12 + vector.y * transform.m22 + vector.z * transform.m32;
    result.z = vector.x * transform.m13 + vector.y * transform.m23 + vector.z * transform.m33;
    return result;
}

/**
 * @brief Converts matrix3 to euler angles tilt pan and roll
 * 
 * @param m 
 * @return EulerAngles 
 */
static EulerAngles convertMatrixToEuler(Matrix3 m)
{
    EulerAngles euler = {0};
    float heading, attitude, bank;
    if (m.m21 > 0.998)
    { // singularity at north pole
        heading = atan2(m.m13, m.m33);
        attitude = PI / 2;
        bank = 0;
        euler.tilt = bank;
        euler.pan = heading * static_cast<float>(RadiansInDegrees);
        euler.roll = attitude * static_cast<float>(RadiansInDegrees);
        return euler;
    }
    if (m.m21 < -0.998)
    { // singularity at south pole
        heading = atan2(m.m13, m.m33);
        attitude = -PI / 2;
        bank = 0;
        euler.tilt = bank;
        euler.pan = heading * static_cast<float>(RadiansInDegrees);
        euler.roll = attitude * static_cast<float>(RadiansInDegrees);

        return euler;
    }
    heading = atan2(-m.m31, m.m11);
    bank = atan2(-m.m23, m.m22);
    attitude = asin(m.m21);
    euler.tilt = bank * static_cast<float>(RadiansInDegrees);
    euler.pan = heading * static_cast<float>(RadiansInDegrees);
    euler.roll = attitude * static_cast<float>(RadiansInDegrees);
    return euler;
}

//Pose operators--------------------------------------------------

/**
 * @brief Operator to change coordinate systems
 * 
 * @param left 
 * @param right 
 * @return PoseData 
 */
static PoseData operator*(PoseData left, PoseData right)
{

    PoseData result;
    result.rotationMatrix = left.rotationMatrix * right.rotationMatrix;
    result.position = transformCoordinate(right.position, left.rotationMatrix) + left.position;
    return result;
}

/**
 * @brief Transform to PoseData column-major matrix and translation
 * 
 * @param rotation 
 * @param translation 
 * @return PoseData 
 */
static PoseData transformToPoseStructure(const float rotation[9], const float translation[3], bool isColumnMajor)
{
    PoseData result;
    result.rotationMatrix = convertArrayToMatrix3(rotation, isColumnMajor);

    result.position.x = translation[0];
    result.position.y = translation[1];
    result.position.z = translation[2];

    result.eulerRotation = convertMatrixToEuler(result.rotationMatrix);
    return result;
}
/**
 * @brief Transform to PoseData from rotation and translation
 * 
 * @param rotation 
 * @param translation 
 * @param isColumnMajor 
 * @return PoseData 
 */
static PoseData transformToPoseStructure(const double rotation[9], const double translation[3], bool isColumnMajor)
{
    PoseData result;

    result.rotationMatrix = convertArrayToMatrix3(rotation, isColumnMajor);

    result.position.x = static_cast<float>(translation[0]);
    result.position.y = static_cast<float>(translation[1]);
    result.position.z = static_cast<float>(translation[2]);

    result.eulerRotation = convertMatrixToEuler(result.rotationMatrix);

    return result;
}

/**
 * @brief Transforms rs2_pose data to PoseData structure (transpose rotation to get rotation from camera to origin coord system)
 * WARNING: Transpose matrix after quaternion conversion based on https://github.com/IntelRealSense/librealsense/blob/master/examples/pose-apriltag/rs-pose-apriltag.cpp
 * @param quaternion 
 * @param translation 
 * @return PoseData 
 */
static PoseData transformToPosestructure(const rs2_pose &pose, bool isColumnMajor)
{
    PoseData tf;
    tf.rotationMatrix = quaternionToMatrix(pose.rotation);

    //? get rotation to original coordinate system not the actual rotation of camera
    if (isColumnMajor)
    {

        tf.rotationMatrix = transpose(tf.rotationMatrix);
    }

    tf.position.x = pose.translation.x;
    tf.position.y = pose.translation.y;
    tf.position.z = pose.translation.z;
    return tf;
}

#endif