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
#include <apriltag/apriltag_pose.h>

/**
 * @brief Structure to store euler angles
 * 
 */
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

/**
 * @brief Structure to store all the pose data from an object
 * 
 */
struct PoseData
{
    Matrix3 rotationMatrix;
    Vector3 position;
    EulerAngles eulerRotation;
};
/**
 * @brief Structure to keep all tag data
 * 
 */
struct TagStructure
{
    PoseData *tagsCameraPositions;
    PoseData *tagsWorldPositions;
    int totalTagsDetected;
};

//Debug prints-----------------------------------------

static void printEulers(EulerAngles angles)
{
    std::cout << "------------------Euler angles-------------------------\ntilt: " << angles.tilt << "\n";
    std::cout << "pan: " << angles.pan << "\n";
    std::cout << "roll: " << angles.roll << "\n";
    std::cout << "-------------------------------------------------------"
              << std::endl;
}




#endif