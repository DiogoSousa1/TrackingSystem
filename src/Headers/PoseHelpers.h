/**
 * @file PoseHelpers.h
 * @author Diogo Sousa
 * @brief All pose structure operators
 * @version 1.0
 * @date 2021-08-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef POSEHELPERS_H_
#define POSEHELPERS_H_
#include <librealsense2/rsutil.h>
#include "TrackingStructures.h"
#include <math.h>

//My headers
#include "MatrixHelpers.h"
#include "QuaternionHelpers.h"
#include "VectorHelpers.h"

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
    result.rotationMatrix = right.rotationMatrix * left.rotationMatrix;
    result.position = transformCoordinate(right.position, left.rotationMatrix) + left.position;

    result.eulerRotation = convertMatrixToEuler(result.rotationMatrix);
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

    if (isColumnMajor)
    {

        tf.rotationMatrix = transpose(tf.rotationMatrix);
    }

    tf.position.x = pose.translation.x;
    tf.position.y = pose.translation.y;
    tf.position.z = pose.translation.z;
    tf.eulerRotation = convertMatrixToEuler(tf.rotationMatrix);
    return tf;
}

static void printPoseData(PoseData data)
{
    printVector3(data.position);
    printEulers(data.eulerRotation);
}

static void printApriltagRawData(apriltag_pose_t &pose)
{
    std::cout << "--------------------April tag pose raw------------------\nRotation:\n";
    std::cout << "11: " << pose.R->data[0] << "  12: " << pose.R->data[1] << "  13: " << pose.R->data[2] << "\n";
    std::cout << "21: " << pose.R->data[3] << "  22: " << pose.R->data[4] << "  23: " << pose.R->data[5] << "\n";
    std::cout << "31: " << pose.R->data[6] << "  32: " << pose.R->data[7] << "  33: " << pose.R->data[8] << "\n";
    printEulers(convertMatrixToEuler(convertArrayToMatrix3(pose.R->data, false)));

    std::cout << "Translation\nx: " << pose.t->data[0] << " y: " << pose.t->data[1] << " z: " << pose.t->data[0] << std::endl;
}

#endif