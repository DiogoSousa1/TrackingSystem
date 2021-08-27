/**
 * @file TagManager.h
 * @author Diogo Sousa
 * @brief Tag manager responsible for detecting tags and storing all their data
 * @version 1.0
 * @date 2021-08-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
/* TAGMANAGER_H */
#ifndef TAGMANAGER_H_
#define TAGMANAGER_H_

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#include <memory>
#include <iostream>

//My headers
#include "MathHelper.h"
#include "PoseHelpers.h"
#include "TrackingStructures.h"

class Tag_Manager
{
public:
	//constructor and destructors
	Tag_Manager(const rs2_extrinsics extrinsics, const rs2_intrinsics &intrisics, float tagSize);
	virtual ~Tag_Manager();

	/**
	 * @brief Detects tag in image from fisheye lens of t265
	 * 
	 * @param grayImage the bytes of the image
	 * @param camera_world_pose the world pose of the camera in the moment of the detection
	 * @return true if tag was detected
	 * @return false if no tag was detected
	 */
	bool detect(unsigned char *grayImage, const rs2_pose *camera_world_pose);


	static void apriltag_pose_destroy(apriltag_pose_t *p)
	{
		matd_destroy(p->R);
		matd_destroy(p->t);
		delete p;
	}

	TagStructure allTagsDetected;

private:
	apriltag_detector *tag_detector;
	apriltag_family_t *tag;
	apriltag_detection_info_t info;
	rs2_intrinsics camera_intrinsics;
	PoseData body_to_Fisheye_data;

	/**
	 * @brief Computes tag pose in camera world coord system using the camera world pose when detection happened
	 * 
	 * @param tagData the tags pose data relative to camera
	 * @param camera_world_pose the world pose of the camera in the moment of the detection 
	 * @return PoseData 
	 */
	PoseData compute_tag_pose_in_world(PoseData tagData, const rs2_pose &camera_world_pose);
};

#endif /* TAGMANAGER_H_ */
