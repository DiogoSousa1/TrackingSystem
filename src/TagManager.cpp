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
}

 
bool Tag_Manager::detect(unsigned char *image)
{
    image_u8_t img = {camera_intrinsics.width, camera_intrinsics.height, camera_intrinsics.width, image};
    
    allTagsDetected.detections = std::shared_ptr<zarray_t>(apriltag_detector_detect(td, &img), apriltag_detections_destroy);
    int totalTagsDetected = zarray_size(allTagsDetected.detections.get());
    if(!totalTagsDetected) {
        return false;
    }
    allTagsDetected.tagsPositions.resize(totalTagsDetected);
    allTagsDetected.pose_raw.resize(totalTagsDetected);
    auto info_ = info;

    //right now only one tag is important
    for (int actualTag = 0; actualTag < totalTagsDetected; actualTag++)
    {
        allTagsDetected.pose_raw[actualTag] = std::shared_ptr<apriltag_pose_t>(new apriltag_pose_t(), apriltag_pose_destroy);
        
        undistort(*dataDetection, camera_intrinsics);

    //TODO: seg fault here
        estimate_pose_for_tag_homography(&info, rawPose);
        std::cout << "Arrived here!";

        for (int c : {1, 2, 4, 5, 7, 8})
        {
            rawPose->R->data[c] *= -1;
        }

        cameraCoordinatesPosition = transformToRS2Structure(rawPose->R->data, rawPose->t->data);
        allTagsDetected.tagsPositions[actualTag] = cameraCoordinatesPosition;
        allTagsDetected.eulerOftags[actualTag] = convertMatrixToEuler(cameraCoordinatesPosition.rotation);
    }
    apriltag_detection_destroy(dataDetection);
    image_u8_destroy(&img);
    return true;
}