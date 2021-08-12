#include "Headers/TagManager.h"

Tag_Manager::Tag_Manager(const rs2_extrinsics extrinsics, const rs2_intrinsics &intrisics, float tagSize)
{
    tag_detector = apriltag_detector_create();
    tag = tag36h11_create();
    apriltag_detector_add_family(tag_detector, tag);
    camera_intrinsics = intrisics;
    tag_detector->quad_decimate = 1.0f;
    tag_detector->quad_sigma = 0.0f;
    tag_detector->nthreads = 1;
    tag_detector->debug = 0;
    tag_detector->refine_edges = 1;
    allTagsDetected = {0};
    info.tagsize = tagSize;
    info.fx = info.fy = 1;
    info.cx = info.cy = 0;
}

Tag_Manager::~Tag_Manager()
{
    apriltag_detector_destroy(tag_detector);
    tag36h11_destroy(tag);
    free(&info);
    free(&allTagsDetected);
    delete this;
}

 
bool Tag_Manager::detect(unsigned char *image)
{
    image_u8_t img = {camera_intrinsics.width, camera_intrinsics.height, camera_intrinsics.width, image};
    zarray_t *detection = apriltag_detector_detect(tag_detector, &img);
    int totalTagsDetected = zarray_size(detection);
    if(!totalTagsDetected) {  
        return false;
    }

    free(allTagsDetected.tagsPositions);
    allTagsDetected.totalTagsDetected = totalTagsDetected;

    //alloc memory for tag data
    allTagsDetected.tagsPositions = (PoseData*) malloc(sizeof(PoseData)*totalTagsDetected);

    apriltag_detection *dataDetection;
    apriltag_pose_t rawPose;
    PoseData cameraCoordinatesPosition;
    auto info_ = info;
    
    //right now only one tag is important
    for (int actualTag = 0; actualTag < totalTagsDetected; actualTag++)
    {
        zarray_get(detection, actualTag, &dataDetection);
        info_.det = dataDetection;
        undistort(*dataDetection, camera_intrinsics);

    //TODO: seg fault here
        estimate_pose_for_tag_homography(&info_, &rawPose);

        for (int c : {1, 2, 4, 5, 7, 8})
        {
            rawPose.R->data[c] *= -1;
        }
        cameraCoordinatesPosition = transformToPoseStructure(rawPose.R->data, rawPose.t->data);
        allTagsDetected.tagsPositions[actualTag] = cameraCoordinatesPosition;
    }
    apriltag_detection_destroy(dataDetection);
    
    return true;
}