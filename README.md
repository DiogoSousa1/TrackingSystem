# TrackingSystem

<h1>Intel realsense tracking system with tag detection</h1>
<h2>Requirements:</h2>

<ul>
  <li><a href="https://www.intelrealsense.com/tracking-camera-t265/">T265 intel realsense tracking camera model</a></li>
  <li>Linux based system for now</li>
  <li><a href="https://github.com/AprilRobotics/apriltag">AprilTag library</a> installed (not built in windows yet)</li>
  <li><a href="https://github.com/IntelRealSense/librealsense">Librealsense2</a> installed</li>
</ul>

This setup was made using vs code.
This system is being created for wTVision and this rep should be deleted soon!

<h3>Known issues and objectives:</h3>
<ul>
<li>Receive tag and rotation and apply to all the points given by camera (partially done)</li>
<li>World tag rotation not giving stable coordinate system when using natively</li>
<li>ASAP linux support for this project in wTVision - AprilTag library build in windows?</li>
<li>Connect socket to engine ✅</li>
</ul>