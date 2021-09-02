/**
 * @file QuaternionHelpers.h
 * @author Diogo Sousa (diogo.sousa@wtvision.com)
 * @brief All quaternion operators
 * @version 1.0
 * @date 2021-08-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef QUATERNIONHELPERS_H_
#define QUATERNIONHELPERS_H_

//My headers
#include "TrackingStructures.h"
#include "MathHelper.h"

/**
 * @brief 
 * @pre Matrix is represents pure rotation
 * @param m  
 * @return Quaternion 
 */
static Quaternion convertMatrix3ToQuaternion(Matrix3 matrix)
{
    float t;
    Quaternion result = {0};
    if (matrix.m33 < 0)
    {
        if (matrix.m11 > matrix.m22)
        {
            t = 1 + matrix.m11 - matrix.m22 - matrix.m33;
            result.x = t;
            result.y = matrix.m12 + matrix.m21;
            result.z = matrix.m31 + matrix.m13;
            result.w = matrix.m23 - matrix.m32;
        }
        else
        {
            t = 1 - matrix.m11 + matrix.m22 - matrix.m33;
            result.x = matrix.m12 + matrix.m21;
            result.y = t;
            result.z = matrix.m23 + matrix.m32;
            result.w = matrix.m31 - matrix.m13;
        }
    }
    else
    {
        if (matrix.m11 < -matrix.m22)
        {
            t = 1 - matrix.m11 - matrix.m22 + matrix.m33;
            result.x = matrix.m31 + matrix.m13;
            result.y = matrix.m23 + matrix.m32;
            result.z = t;
            result.w = matrix.m12 - matrix.m21;
        }
        else
        {
            t = 1 + matrix.m11 + matrix.m22 + matrix.m33;
            result.x = matrix.m23 - matrix.m32;
            result.y = matrix.m31 - matrix.m13;
            result.z = matrix.m12 - matrix.m21;
            result.w = t;
        }
    }
    result.x *= 0.5f/sqrt(t);
    result.y *= 0.5f/sqrt(t);
    result.z *= 0.5f/sqrt(t);
    result.w *= 0.5f/sqrt(t);
    return result;
}

static Quaternion multiply(Quaternion left, Quaternion right)
{

    Quaternion result = {0};
    float lx = left.x;
    float ly = left.y;
    float lz = left.z;
    float lw = left.w;
    float rx = right.x;
    float ry = right.y;
    float rz = right.z;
    float rw = right.w;
    float a = (ly * rz - lz * ry);
    float b = (lz * rx - lx * rz);
    float c = (lx * ry - ly * rx);
    float d = (lx * rx + ly * ry + lz * rz);
    result.x = (lx * rw + rx * lw) + a;
    result.y = (ly * rw + ry * lw) + b;
    result.z = (lz * rw + rz * lw) + c;
    result.w = lw * rw - d;
    return result;
}

static Quaternion rotationPanTiltRoll(float pan, float tilt, float roll)
{
    Quaternion result = {0};
    float halfRoll = roll * 0.5f;
    float halfPitch = tilt * 0.5f;
    float halfYaw = pan * 0.5f;

    float sinRoll = sin(halfRoll);
    float cosRoll = cos(halfRoll);
    float sinPitch = sin(halfPitch);
    float cosPitch = cos(halfPitch);
    float sinYaw = sin(halfYaw);
    float cosYaw = cos(halfYaw);

    result.x = (cosYaw * sinPitch * cosRoll) + (sinYaw * cosPitch * sinRoll);
    result.y = (sinYaw * cosPitch * cosRoll) - (cosYaw * sinPitch * sinRoll);
    result.z = (cosYaw * cosPitch * sinRoll) - (sinYaw * sinPitch * cosRoll);
    result.w = (cosYaw * cosPitch * cosRoll) + (sinYaw * sinPitch * sinRoll);
    return result;
}

static Quaternion IdentityQuaternion()
{
    Quaternion result = {0};
    result.x = result.y = result.z = 0;
    result.w = 1.0f;
    return result;
}

static Quaternion rotateQuaternionZ(float angle)
{
    Quaternion result = {0};
    float half = angle * 0.5f;
    float sinVal = sin(half);
    float cosVal = cos(half);
    result.x = 0;
    result.y = 0;
    result.z = 1.0f * sinVal;
    result.w = cosVal;
    return result;
}

static Quaternion operator*(Quaternion left, Quaternion right)
{
    return multiply(left, right);
}

static float LengthSquareOfQuaternion(Quaternion q)
{
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

static float MagnitudeOfQuaternion(Quaternion q)
{
    return sqrt(LengthSquareOfQuaternion(q));
}

static Quaternion invertQuaternion(Quaternion q)
{
    float lengthSq = LengthSquareOfQuaternion(q);
    Quaternion result = {0};
    if (abs(lengthSq) > 0.00001f)
    {

        lengthSq = 1.0f / lengthSq;

        result.x = -q.x * lengthSq;
        result.y = -q.y * lengthSq;
        result.z = -q.z * lengthSq;
        result.w = q.w * lengthSq;
        return result;
    }
    else
    {
        fprintf(stderr, "Length of quaternion is 0!");
    }
    return result;
}
static Quaternion convertEulerToQuaternion(EulerAngles a)
{
    Quaternion result = {0};
    float c1 = cos(degreesToRadians(a.pan) / 2.0f);
    float s1 = sin(degreesToRadians(a.pan) / 2.0f);
    float c2 = cos(degreesToRadians(a.roll) / 2.0f);
    float s2 = sin(degreesToRadians(a.roll) / 2.0f);
    float c3 = cos(degreesToRadians(a.tilt) / 2.0f);
    float s3 = sin(degreesToRadians(a.tilt) / 2.0f);
    float c1c2 = c1 * c2;
    float s1s2 = s1 * s2;
    result.w = c1c2 * c3 - s1s2 * s3;
    result.x = c1c2 * s3 + s1s2 * c3;
    result.y = s1 * c2 * c3 + c1 * s2 * s3;
    result.z = c1 * s2 * c3 - s1 * c2 * s3;
    return result;
}
static EulerAngles convertQuaternionToEuler(Quaternion q)
{
    EulerAngles result = {0};
    double test = q.x * q.y + q.z * q.w;
    double heading, attitude, bank;

    if (test > 0.49999)
    { // singularity at north pole
        heading = 2 * atan2(q.x, q.w);
        attitude = PI / 2;
        bank = 0;
    }
    else if (test < -0.49999)
    { // singularity at south pole
        heading = -2 * atan2(q.x, q.w);
        attitude = -PI / 2;
        bank = 0;
    }
    else
    {
        double sqx = q.x * q.x;
        double sqy = q.y * q.y;
        double sqz = q.z * q.z;
        heading = atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
        attitude = asin(2 * test);
        bank = atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);
    }
    result.tilt = bank * RadiansInDegrees;
    result.pan = heading * RadiansInDegrees;
    result.roll = attitude * RadiansInDegrees;
    return result;
}

static void printQuaternion(Quaternion q)
{
    std::cout << "--------------Quaternion with values------------------------\n";
    std::cout << "x: " << q.x << " y: " << q.y << " z: " << q.z << " w: " << q.w << std::endl;
}

#endif