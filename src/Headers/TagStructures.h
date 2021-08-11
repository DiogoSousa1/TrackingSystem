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

struct TagStructure
{
    rs2_extrinsics* tagsPositions;
    int totalTagsDetected;
    EulerAngles* eulerOftags;
};


#endif