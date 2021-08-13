/**
 * @file TagStructures.h
 * @author Diogo Sousa 
 * @brief File where all structure definitions and debuggers are declared
 * @version 1.0
 * @date 2021-08-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef TAGSTRUCTURES_H
#define TAGSTRUCTURES_H
#include <librealsense2/rsutil.h>
#include <iostream>

struct EulerAngles
{
    float tilt;
    float pan;
    float roll;
};

/**
 * @brief Representation of a matrix in format 
 *  m11 m12 m13
 *  m21 m22 m23
 *  m31 m32 m33
 * 
 */
struct Matrix3
{
    float m11;
    float m12;
    float m13;

    float m21;
    float m22;
    float m23;

    float m31;
    float m32;
    float m33;
};

typedef rs2_vector Vector3;
typedef rs2_quaternion Quaternion;

struct PoseData
{
    Matrix3 rotationMatrix;
    Vector3 position;
    EulerAngles eulerRotation;
};

struct TagStructure
{
    PoseData *tagsCameraPositions;
    PoseData *tagsWorldPositions;
    int totalTagsDetected;
};

static void printEulers(EulerAngles angles)
{
    std::cout << "------------------Euler angles-------------------------\ntilt: " << angles.tilt << "\n";
    std::cout << "pan: " << angles.pan << "\n";
    std::cout << "roll: " << angles.roll << "\n";
    std::cout << "-------------------------------------------------------"
              << std::endl;
}

static void printVector3(Vector3 vector)
{

    std::cout << "--------------Vector with values-----------------------\nx: " << vector.x << "\n";
    std::cout << "y: " << vector.y << "\n";
    std::cout << "z: " << vector.z << "\n";
    std::cout << "-------------------------------------------------------"
              << std::endl;
}

static void printPoseData(PoseData data)
{
    printVector3(data.position);
    printEulers(data.eulerRotation);
    //print of matrix is useless in terms of debug
}

#endif