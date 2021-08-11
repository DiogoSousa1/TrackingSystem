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
#include <vector>
struct EulerAngles {
    float x;
    float y;
    float z;
};

struct TagStructure
{
    std::shared_ptr<zarray_t> detections;
    std::vector<std::shared_ptr<apriltag_pose_t>> pose_raw;
    std::vector<std::shared_ptr<rs2_extrinsics>> tagsPositions;
    int totalTagsDetected;
    EulerAngles* eulerOftags;
};


#endif