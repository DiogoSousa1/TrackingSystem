# TrackingSystem

<h1>Intel realsense tracking system with tag detection</h1>
<h2>Requirements:</h2>

<ul>
  <li><a href="https://www.intelrealsense.com/tracking-camera-t265/">T265 intel realsense tracking camera model</a></li>
  <li><a href="https://github.com/AprilRobotics/apriltag">AprilTag library</a> installed (not built in windows yet)</li>
  <li><a href="https://github.com/IntelRealSense/librealsense">Librealsense2</a> installed</li>
</ul>

This setup was made using vs code.
This system is being created for wTVision and this rep should be deleted soon!

<h3>Known issues and objectives:</h3>
<ul>
<li>Windows support for this project in wTVision - AprilTag library build</li>
<li>Increase extensibility of the program</li>
<li>Use quaternions to all calculations of rotation</li>
</ul>

<h3>Working branches</h3>
<p>Main branch uses matrix calculations for rotations and sends quaternions to engine to avoid gimbal lock</p>
<p>Quaternion branch uses only quaternion calculations to represent rotation (not working properly)</p>
