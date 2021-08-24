#include "Headers/TrackingDevice.h"

TrackingDevice::TrackingDevice(EngineClient &engine_client) : client(engine_client)
{
    stop = false;
}
TrackingDevice::~TrackingDevice()
{
}

void TrackingDevice::startTracking(const float tagSize)
{
    rs2::pipeline camPipeline;
    rs2::config cfg;
    cfg.enable_all_streams();

    const int fisheye_sensor_idx = 1;
    rs2::pipeline_profile profile = camPipeline.start(cfg);
    rs2::stream_profile fisheyeStream = profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    rs2_extrinsics tagPose = {0};
    rs2_intrinsics fisheye_intrinsics = fisheyeStream.as<rs2::video_stream_profile>().get_intrinsics();
    rs2_extrinsics body_toFisheye_extrinsics = fisheyeStream.get_extrinsics_to(profile.get_stream(RS2_STREAM_POSE));

    Tag_Manager tagManager = Tag_Manager(body_toFisheye_extrinsics, fisheye_intrinsics, tagSize);

    while (!stop)
    {

        rs2::frameset frame = camPipeline.wait_for_frames();
        rs2::video_frame fisheyeFrame = frame.get_fisheye_frame(fisheye_sensor_idx);
        unsigned long long frame_Number = fisheyeFrame.get_frame_number();
        rs2_pose cameraLastKnownPose;
        rs2::pose_frame poseFrame = frame.get_pose_frame();
        rs2_pose lastPose = poseFrame.get_pose_data();

        cout << "Tracker confidence: " << lastPose.tracker_confidence << "\n";
        //only do tag detector between 6 frames
        if (frame_Number % 6 == 0)
        {
            fisheyeFrame.keep();

            if (tagManager.detect((unsigned char *)fisheyeFrame.get_data(), &lastPose))
            {

                cameraLastKnownPose = lastPose;
            }
        }

        if (tagManager.allTagsDetected.totalTagsDetected > 0)
        {

            PoseData tagWorldPose = tagManager.allTagsDetected.tagsWorldPositions[0];
            PoseData tagCameraPose = tagManager.allTagsDetected.tagsCameraPositions[0];
            cout << "Tag pose in camera:\n";
            printPoseData(tagCameraPose);

            cout << "--------------------------\n\nTag pose in world:\n";
            printPoseData(tagWorldPose);
            printMatrix3(tagWorldPose.rotationMatrix);

            //need rotations to align y with tags normal
            Matrix3 coordinateTransform = rotateX(degreesToRadians(-90.0f)) * transpose(tagWorldPose.rotationMatrix);

            //invert z to left hand coord system
            coordinateTransform.m13 = -coordinateTransform.m13;
            coordinateTransform.m23 = -coordinateTransform.m23;
            coordinateTransform.m33 = -coordinateTransform.m33;

            cout << "World coordinate transformation:\n";
            printMatrix3(coordinateTransform);
            printEulers(convertMatrixToEuler(coordinateTransform));
            PoseData enginePose = {0};
            enginePose.position = transformCoordinate((lastPose.translation - tagWorldPose.position), coordinateTransform);

            Matrix3 cameraRotation = coordinateTransform * transpose(quaternionToMatrix(lastPose.rotation));
            enginePose.rotationMatrix = cameraRotation;

            enginePose.eulerRotation = convertMatrixToEuler(enginePose.rotationMatrix);
            enginePose.eulerRotation.tilt *= -1.0f;
            cout << "------------------------------\n\nSending to engine:\n";
            printPoseData(enginePose);

            client.sendToEngine(enginePose);
        }
        else
        {
            cout << "Waiting for tag detection..." << endl;
        }

        lseek(1, 0, SEEK_SET);
    }

    camPipeline.stop();
}

void TrackingDevice::stopTracking()
{
    stop = true;
}