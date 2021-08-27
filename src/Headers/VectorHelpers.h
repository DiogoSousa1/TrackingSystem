/**
 * @file VectorHelpers.h
 * @author Diogo Sousa (diogo.sousa@wtvision.com)
 * @brief All vector operators
 * @version 1.0
 * @date 2021-08-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef VECTORHELPERS_H_
#define VECTORHELPERS_H_

//My headers
#include "TrackingStructures.h"

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

static Vector3 rotateVector(Vector3 vector, Quaternion rotation)
{
    Vector3 result = {0};
    float x = rotation.x + rotation.x;
    float y = rotation.y + rotation.y;
    float z = rotation.z + rotation.z;
    float wx = rotation.w * x;
    float wy = rotation.w * y;
    float wz = rotation.w * z;
    float xx = rotation.x * x;
    float xy = rotation.x * y;
    float xz = rotation.x * z;
    float yy = rotation.y * y;
    float yz = rotation.y * z;
    float zz = rotation.z * z;

    result.x = ((vector.x * ((1.0f - yy) - zz)) + (vector.y * (xy - wz))) + (vector.z * (xz + wy));
    result.y = ((vector.x * (xy + wz)) + (vector.y * ((1.0f - xx) - zz))) + (vector.z * (yz - wx));
    result.z = ((vector.x * (xz - wy)) + (vector.y * (yz + wx))) + (vector.z * ((1.0f - xx) - yy));
    return result;
}

static void printVector3(Vector3 vector)
{

    std::cout << "--------------Vector with values-----------------------\nx: " << vector.x << "\n";
    std::cout << "y: " << vector.y << "\n";
    std::cout << "z: " << vector.z << "\n";
    std::cout << "-------------------------------------------------------"
              << std::endl;
}


#endif