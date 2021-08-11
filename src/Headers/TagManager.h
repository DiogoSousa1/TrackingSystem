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
#include "GeometryHelpers.h"
#include "TagStructures.h"

class Tag_Manager
{
public:

	//constructor and destructors
	Tag_Manager(const rs2_extrinsics extrinsics, const rs2_intrinsics &intrisics, float tagSize);
	virtual ~Tag_Manager();

	//detect tag position and rotation relative to camera
	bool detect(unsigned char *grayImage);

	static void apriltag_pose_destroy(apriltag_pose_t* p){ matd_destroy(p->R); matd_destroy(p->t); delete p;}
   
	TagStructure allTagsDetected;
	apriltag_detector *tag_detector;
	apriltag_family_t *tag;
	apriltag_detection_info_t info;
	rs2_intrinsics camera_intrinsics;

};

#endif /* TAGMANAGER_H_ */
