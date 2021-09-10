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
    body_to_Fisheye_data = transformToPoseStructure(extrinsics.rotation, extrinsics.translation, true);
}

Tag_Manager::~Tag_Manager()
{
    apriltag_detector_destroy(tag_detector);
    tag36h11_destroy(tag);
    info = {0};
    allTagsDetected = {0};
}

bool Tag_Manager::detect(unsigned char *image, const rs2_pose *camera_world_pose)
{
    image_u8_t img = {camera_intrinsics.width, camera_intrinsics.height, camera_intrinsics.width, image};
    zarray_t *detection = apriltag_detector_detect(tag_detector, &img);
    int totalTagsDetected = zarray_size(detection);
    if (totalTagsDetected == 0)
    {
        return false;
    }

    free(allTagsDetected.tagsCameraPositions);
    free(allTagsDetected.tagsWorldPositions);
    allTagsDetected.totalTagsDetected = totalTagsDetected;

    //alloc memory for tag data
    allTagsDetected.tagsCameraPositions = (PoseData *)malloc(sizeof(PoseData) * totalTagsDetected);
    allTagsDetected.tagsWorldPositions = (PoseData *)malloc(sizeof(PoseData) * totalTagsDetected);

    apriltag_detection *dataDetection = {0};
    apriltag_pose_t rawPose;
    PoseData cameraCoordinatesPosition;

    //right now only one tag is important
    for (int actualTag = 0; actualTag < totalTagsDetected; actualTag++)
    {
        zarray_get(detection, actualTag, &dataDetection);
        info.det = dataDetection;
        undistort(*dataDetection, camera_intrinsics);

        estimate_pose_for_tag_homography(&info, &rawPose);

        for (int c : {1, 2, 4, 5, 7, 8})
        {
            rawPose.R->data[c] *= -1;
        }
        //transpose rawPose.R to get rotation from tag to camera
        cameraCoordinatesPosition = transformToPoseStructure(rawPose.R->data, rawPose.t->data, true);
        allTagsDetected.tagsCameraPositions[actualTag] = cameraCoordinatesPosition;

        allTagsDetected.tagsWorldPositions[actualTag] = compute_tag_pose_in_world(cameraCoordinatesPosition, *camera_world_pose);
    }
    apriltag_detection_destroy(dataDetection);

    return true;
}

PoseData Tag_Manager::compute_tag_pose_in_world(PoseData cameraTagData, const rs2_pose &camera_world_pose)
{
    PoseData worldTagData = {0};
    PoseData world_to_body = transformToPosestructure(camera_world_pose, true);
    //compute tag rotation and translation from world coord system to tag
    worldTagData = world_to_body * body_to_Fisheye_data * cameraTagData;
    return worldTagData;
}