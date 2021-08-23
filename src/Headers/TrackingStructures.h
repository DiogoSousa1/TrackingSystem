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

static void printMatrix3(Matrix3 matrix)
{
    std::cout << "--------------Matrix with values------------------------------\n";
    std::cout << "11: " << matrix.m11 << "  12: " << matrix.m12 << "  13: " << matrix.m13 << "\n";
    std::cout << "21: " << matrix.m21 << "  22: " << matrix.m22 << "  23: " << matrix.m23 << "\n";
    std::cout << "31: " << matrix.m31 << "  32: " << matrix.m32 << "  33: " << matrix.m33 << "\n";
    std::cout << "-------------------------------------------------------" << std::endl;
}

static void printApriltagRawData(apriltag_pose_t &pose)
{
    std::cout << "--------------------April tag pose raw------------------\nRotation:\n";
    std::cout << "11: " << pose.R->data[0] << "  12: " << pose.R->data[1] << "  13: " << pose.R->data[2] << "\n";
    std::cout << "21: " << pose.R->data[3] << "  22: " << pose.R->data[4] << "  23: " << pose.R->data[5] << "\n";
    std::cout << "31: " << pose.R->data[6] << "  32: " << pose.R->data[7] << "  33: " << pose.R->data[8] << "\n";
    std::cout << "Translation\nx: " << pose.t->data[0] << " y: " << pose.t->data[1] << " z: " << pose.t->data[0] << std::endl;
}

#endif