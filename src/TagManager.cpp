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

    info->tagsize = tagSize;
    info->fx = info->fy = 1;
    info->cx = info->cy = 0;
}

Tag_Manager::~Tag_Manager()
{
    apriltag_detector_destroy(tag_detector);
    tag36h11_destroy(tag);
    info = nullptr;
}

bool Tag_Manager::detect(unsigned char *image, TagStructure *tags)
{
    image_u8_t img = {camera_intrinsics.width, camera_intrinsics.height, camera_intrinsics.width, image};
    zarray_t *detection = apriltag_detector_detect(tag_detector, &img);
    int totalTagsDetected = zarray_size(detection);
    if(!totalTagsDetected) {    
        return false;
    }
    free(tags->eulerOftags);
    free(tags->tagsPositions);
    tags->totalTagsDetected = totalTagsDetected;
    tags->tagsPositions = (rs2_extrinsics*) malloc(sizeof(rs2_extrinsics)*totalTagsDetected);
    tags->eulerOftags = (EulerAngles*) malloc(sizeof(EulerAngles)*totalTagsDetected);
    apriltag_detection *dataDetection;
    apriltag_pose_t *rawPose = new apriltag_pose_t();
    rs2_extrinsics cameraCoordinatesPosition;

    //right now only one tag is important
    for (int actualTag = 0; actualTag < totalTagsDetected; actualTag++)
    {
        zarray_get(detection, actualTag, dataDetection);

        undistort(*dataDetection, camera_intrinsics);

        estimate_pose_for_tag_homography(info, rawPose);

        for (int c : {1, 2, 4, 5, 7, 8})
        {
            rawPose->R->data[c] *= -1;
        }

        cameraCoordinatesPosition = transformToRS2Structure(rawPose->R->data, rawPose->t->data);
        tags->tagsPositions[actualTag] = cameraCoordinatesPosition;
        tags->eulerOftags[actualTag] = convertMatrixToEuler(cameraCoordinatesPosition.rotation);
    }

    apriltag_detection_destroy(dataDetection);
    image_u8_destroy(&img);
    return true;
}
