/**
 * @file TagStructures.h
 * @author Diogo Sousa 
 * @brief File where all structure definitions are declared
 * @version 1.0
 * @date 2021-08-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef TAGSTRUCTURES_H
#define TAGSTRUCTURES_H
#include <librealsense2/rsutil.h>


struct EulerAngles {
    float x;
    float y;
    float z;
};

/**
 * @brief Representation of a matrix in format 
 *  m11 m12 m13
 *  m21 m22 m23
 *  m31 m32 m33
 * 
 */
struct Matrix {
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

struct poseData {
    Matrix rotation;
    Vector3 translation;
    EulerAngles eulerOfRotation;
};

struct TagStructure
{
    poseData* tagsPositions;
    int totalTagsDetected;
};

#endif