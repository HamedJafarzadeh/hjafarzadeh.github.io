---
layout: post
title:  Record a ROS bag file of Intel Real sense D435 
categories: robotics
---
# Record a ROS bag file of Intel Real sense D435

If it is your first time, you need to make several configurations, please proceed to [First Time Setup](#first-time-setup).

## Quick check list :

- Open your terminal and enter (I'm using terminator, which gives your better options to manage your terminal windows)

  ```bash
  roscore
  ```
- Open another terminal and run 

 ```bash
  roslaunch realsense2_camera rs_camera_record.launch
  ```

- Open another terminal, navigate to your rviz configuration and run the following command

```bash
  cd thesisproject/realsense/rviz
  rviz pointcloud.rviz
  ```



## First Time Setup

First you need to run `roscore`.

- Open your terminal and enter (I'm using terminator, which gives your better options to manage your terminal windows)

  ```bash
  roscore
  ```

- after couple of seconds you should see following line which shows that ROS core started successfully.

  ~~~bash
  started core service [/rosout]
  ~~~

* now open another terminal (another tab in terminator) and change directory to your catkin source file, in my case it is located in `~\catkin_ws\src` and then change directory again to `~\catkin_ws\src\realsense\realsense2_camera\launch` and enter following command

* ``` bash
  nano rs_camera_record.launch
  ```

* enter the following launch configurations to the newly created file

* ```xml
  <launch>
    <arg name="serial_no"           default=""/>
    <arg name="json_file_path"      default=""/>
    <arg name="camera"              default="rscamera"/>
    <arg name="tf_prefix"           default="$(arg camera)"/>
  
    <arg name="fisheye_width"       default="640"/>
    <arg name="fisheye_height"      default="480"/>
    <arg name="enable_fisheye"      default="true"/>
  
    <arg name="depth_width"         default="640"/>
    <arg name="depth_height"        default="480"/>
    <arg name="enable_depth"        default="true"/>
  
    <arg name="infra_width"        default="640"/>
    <arg name="infra_height"       default="480"/>
    <arg name="enable_infra1"       default="true"/>
    <arg name="enable_infra2"       default="true"/>
  
    <arg name="color_width"         default="640"/>
    <arg name="color_height"        default="480"/>
    <arg name="enable_color"        default="true"/>
  
    <arg name="fisheye_fps"         default="30"/>
    <arg name="depth_fps"           default="30"/>
    <arg name="infra_fps"           default="30"/>
    <arg name="color_fps"           default="30"/>
    <arg name="gyro_fps"            default="400"/>
    <arg name="accel_fps"           default="250"/>
    <arg name="enable_gyro"         default="true"/>
    <arg name="enable_accel"        default="true"/>
  
    <arg name="enable_pointcloud"         default="true"/>
    <arg name="pointcloud_texture_stream" default="RS2_STREAM_COLOR"/>
    <arg name="pointcloud_texture_index"  default="0"/>
  
    <arg name="enable_sync"           default="true"/>
    <arg name="align_depth"           default="false"/>
  
    <arg name="filters"               default=""/>
    <arg name="clip_distance"         default="-2"/>
    <arg name="linear_accel_cov"      default="0.01"/>
    <arg name="initial_reset"         default="false"/>
    <arg name="unite_imu_method"      default=""/>
    <arg name="topic_odom_in"         default="odom_in"/>
    <arg name="calib_odom_file"       default=""/>
    <arg name="publish_odom_tf"       default="true"/>
  
    
    <group ns="$(arg camera)">
      <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
        <arg name="tf_prefix"                value="$(arg tf_prefix)"/>
        <arg name="serial_no"                value="$(arg serial_no)"/>
        <arg name="json_file_path"           value="$(arg json_file_path)"/>
  
        <arg name="enable_pointcloud"        value="$(arg enable_pointcloud)"/>
        <arg name="pointcloud_texture_stream" value="$(arg pointcloud_texture_stream)"/>
        <arg name="pointcloud_texture_index"  value="$(arg pointcloud_texture_index)"/>
        <arg name="enable_sync"              value="$(arg enable_sync)"/>
        <arg name="align_depth"              value="$(arg align_depth)"/>
  
        <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
        <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
        <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>
  
        <arg name="depth_width"              value="$(arg depth_width)"/>
        <arg name="depth_height"             value="$(arg depth_height)"/>
        <arg name="enable_depth"             value="$(arg enable_depth)"/>
  
        <arg name="color_width"              value="$(arg color_width)"/>
        <arg name="color_height"             value="$(arg color_height)"/>
        <arg name="enable_color"             value="$(arg enable_color)"/>
  
        <arg name="infra_width"              value="$(arg infra_width)"/>
        <arg name="infra_height"             value="$(arg infra_height)"/>
        <arg name="enable_infra1"            value="$(arg enable_infra1)"/>
        <arg name="enable_infra2"            value="$(arg enable_infra2)"/>
  
        <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
        <arg name="depth_fps"                value="$(arg depth_fps)"/>
        <arg name="infra_fps"                value="$(arg infra_fps)"/>
        <arg name="color_fps"                value="$(arg color_fps)"/>
        <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
        <arg name="accel_fps"                value="$(arg accel_fps)"/>
        <arg name="enable_gyro"              value="$(arg enable_gyro)"/>
        <arg name="enable_accel"             value="$(arg enable_accel)"/>
  
        <arg name="filters"                  value="$(arg filters)"/>
        <arg name="clip_distance"            value="$(arg clip_distance)"/>
        <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
        <arg name="initial_reset"            value="$(arg initial_reset)"/>
        <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>
        <arg name="topic_odom_in"            value="$(arg topic_odom_in)"/>
        <arg name="calib_odom_file"          value="$(arg calib_odom_file)"/>
        <arg name="publish_odom_tf"          value="$(arg publish_odom_tf)"/>
      </include>
    </group>
  </launch>
  ```

* Using ctrl+x and saving the file, you can exit from nano editor.

* This configuration is based on rs_camera with a little change that force RealSense to make 'Point Cloud' topic by enabling `enable_pointcloud`

* now you should run the following command to connect RealSense to ROS.

* ```bash
  roslaunch realsense2_camera rs_camera_record.launch
  ```

* unless you didn't receive any error, you are good to go and ROS is connected to RealSense. In order to verify your configuration you can use following command in your terminal to see if RealSense topics are published successfully.

* ```bash
  rostopic list | grep rscamera
  ```

* You must receive list a topics related to RealSense camera.

* The next step is to view the camera stream in rviz. change your directory to your desired directory and use this command to make a rviz configuration.

* `nano rs_test.rviz`

* copy these configuration to this file

```xml
  Panels:
    - Class: rviz/Displays
      Help Height: 78
      Name: Displays
      Property Tree Widget:
        Expanded:
          - /Global Options1
          - /Status1
          - /PointCloud21
          - /Image1
          - /Image2
        Splitter Ratio: 0.5
      Tree Height: 393
    - Class: rviz/Selection
      Name: Selection
    - Class: rviz/Tool Properties
      Expanded:
        - /2D Pose Estimate1
        - /2D Nav Goal1
        - /Publish Point1
      Name: Tool Properties
      Splitter Ratio: 0.588679016
    - Class: rviz/Views
      Expanded:
        - /Current View1
      Name: Views
      Splitter Ratio: 0.5
    - Class: rviz/Time
      Experimental: false
      Name: Time
      SyncMode: 0
      SyncSource: PointCloud2
  Toolbars:
    toolButtonStyle: 2
  Visualization Manager:
    Class: ""
    Displays:
      - Alpha: 0.5
        Cell Size: 1
        Class: rviz/Grid
        Color: 160; 160; 164
        Enabled: true
        Line Style:
          Line Width: 0.0299999993
          Value: Lines
        Name: Grid
        Normal Cell Count: 0
        Offset:
          X: 0
          Y: 0
          Z: 0
        Plane: XY
        Plane Cell Count: 10
        Reference Frame: <Fixed Frame>
        Value: true
      - Alpha: 1
        Autocompute Intensity Bounds: true
        Autocompute Value Bounds:
          Max Value: 10
          Min Value: -10
          Value: true
        Axis: Z
        Channel Name: intensity
        Class: rviz/PointCloud2
        Color: 255; 255; 255
        Color Transformer: RGB8
        Decay Time: 0
        Enabled: true
        Invert Rainbow: false
        Max Color: 255; 255; 255
        Max Intensity: 4096
        Min Color: 0; 0; 0
        Min Intensity: 0
        Name: PointCloud2
        Position Transformer: XYZ
        Queue Size: 10
        Selectable: true
        Size (Pixels): 3
        Size (m): 3
        Style: Points
        Topic: /rscamera/depth/color/points
        Unreliable: false
        Use Fixed Frame: true
        Use rainbow: true
        Value: true
      - Class: rviz/Image
        Enabled: true
        Image Topic: /rscamera/color/image_raw
        Max Value: 1
        Median window: 5
        Min Value: 0
        Name: Image
        Normalize Range: true
        Queue Size: 2
        Transport Hint: raw
        Unreliable: false
        Value: true
      - Class: rviz/Image
        Enabled: true
        Image Topic: /rscamera/depth/image_rect_raw
        Max Value: 1
        Median window: 5
        Min Value: 0
        Name: Image
        Normalize Range: true
        Queue Size: 2
        Transport Hint: raw
        Unreliable: false
        Value: true
    Enabled: true
    Global Options:
      Background Color: 48; 48; 48
      Default Light: true
      Fixed Frame: rscamera_link
      Frame Rate: 30
    Name: root
    Tools:
      - Class: rviz/Interact
        Hide Inactive Objects: true
      - Class: rviz/MoveCamera
      - Class: rviz/Select
      - Class: rviz/FocusCamera
      - Class: rviz/Measure
      - Class: rviz/SetInitialPose
        Topic: /initialpose
      - Class: rviz/SetGoal
        Topic: /move_base_simple/goal
      - Class: rviz/PublishPoint
        Single click: true
        Topic: /clicked_point
    Value: true
    Views:
      Current:
        Class: rviz/FPS
        Enable Stereo Rendering:
          Stereo Eye Separation: 0.0599999987
          Stereo Focal Distance: 1
          Swap Stereo Eyes: false
          Value: false
        Invert Z Axis: false
        Name: Current View
        Near Clip Distance: 0.00999999978
        Pitch: 0.520316124
        Position:
          X: -3.55842924
          Y: -1.0189333
          Z: 2.73017335
        Target Frame: camera_color_frame
        Value: FPS (rviz)
        Yaw: 0.259642988
      Saved: ~
  Window Geometry:
    Displays:
      collapsed: false
    Height: 1053
    Hide Left Dock: false
    Hide Right Dock: false
    Image:
      collapsed: false
    QMainWindow State: 000000ff00000000fd0000000400000000000001710000038efc020000000cfb0000001200530065006c0065006300740069006f006e00000001e10000009b0000006200fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000002b0000021b000000da00fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000000c00430061006d006500720061010000038b000000170000000000000000fb0000000c00430061006d00650072006101000003a2000000170000000000000000fb0000000a0049006d006100670065010000024c0000009c0000001700fffffffb0000000a0049006d00610067006501000002ee000000cb0000001700ffffff000000010000010f0000038efc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000002b0000038e000000b200fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007800000003efc0100000002fb0000000800540069006d00650100000000000007800000035200fffffffb0000000800540069006d00650100000000000004500000000000000000000004f40000038e00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
    Selection:
      collapsed: false
    Time:
      collapsed: false
    Tool Properties:
      collapsed: false
    Views:
      collapsed: false
    Width: 1920
    X: 0
    Y: 27
  ```

* Using ctrl+x and saving the file, you can exit from nano editor.

* now you can open rviz with this configuration using the following command :

* `rviz -d rs_test.rviz`

* and you should now be able to see a cloud point of the scene.

* Now, Lets record the topics that are interested for us.

  Following topics are usable in recording dataset

  | Topic name                     | Description                       |
  | ------------------------------ | --------------------------------- |
  | /rscamera/depth/image_rect_raw | stream depth image in `img` type  |
  | /rscamera/color/image_raw      | stream color images in `img` type |
  | /rscamera/infra1/image_rect_raw | infrared camera 1                 |
  | /rscamera/infra2/image_rect_raw | infrared camera 2                 |
  | /rscamera/color/camera_info | Color camera info |
  | /rscamera/depth/camera_info | Depth camera info |
  | /rscamera/extrinsics/depth_to_color | Camera properties |
  | /rscamera/extrinsics/depth_to_infra1 | Camera properties |
  | /rscamera/extrinsics/depth_to_infra2 | Camera properties |
  | /rscamera/depth/color/points | Point cloud in `PointCloud2` type |


* In order to record all of the following topics, we issue the following command .

* ```bash
  rosbag record /rscamera/depth/image_rect_raw /rscamera/color/image_raw /rscamera/infra1/image_rect_raw /rscamera/infra2/image_rect_raw /rscamera/color/camera_info /rscamera/depth/camera_info /rscamera/extrinsics/depth_to_color /rscamera/extrinsics/depth_to_infra1 /rscamera/extrinsics/depth_to_infra2 /rscamera/depth/color/points
  ```

* as soon as you issue the above command, ROS will start record your dataset.

* 
