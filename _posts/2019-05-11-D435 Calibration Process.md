---
layout: post
title:  Realsense D435 Calibration Process using ROS
categories: robotics
---

 **Author**

- Hamed Jafarzadeh - Hamed.Jafarzadeh@Skoltech.ru
- Skolkovo Institue of science and technology - [Mobile Robotics Lab](<http://sites.skoltech.ru/mobilerobotics/>)
- Under supervision of [Dr. Gonzalo Ferrer](<https://faculty.skoltech.ru/people/gonzaloferrer>)

# Camera Calibration

Today’s cheap pinhole cameras introduces a lot of distortion to images. Two major distortions are radial distortion and tangential distortion.

Due to radial distortion, straight lines will appear curved. Its effect is more as we move away from the center of image. For example, one image is shown below, where two edges of a chess board are marked with red lines. But you can see that border is not a straight line and doesn’t match with the red line. All the expected straight lines are bulged out. Visit [Distortion (optics)](http://en.wikipedia.org/wiki/Distortion_(optics)) for more details.



#![](https://opencv-python-tutroals.readthedocs.io/en/latest/_images/calib_radial.jpg)



You can find more details specially on theories behind algorithms on <https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html>.





# Calibration process

- I have used [This](<https://trojan03.github.io/#!/blog/4>) as a start point.

- I have record a bag file of myself, moving a checkboard around, you can refer to my [previous article](<https://hamedjafarzadeh.github.io/robotics/2019/05/04/Record-a-ROS-bag-file-of-Intel-Real-sense-D435.html>) on how to record a bag file.  Later I found that maybe this was a quick way, since a simple 2 min bag file, results in too many pictures that was troublesome for me to pick images from them. maybe you could use rqt_image_viewer instead.

- I faced following buffer issue while recording the bag file :

  `rosbag record buffer exceeded.  Dropping oldest queued message.` 

  so I decided to narrow recording topics to just pictures and I removed the point clouds.

  ``` bash
  rosbag record /rscamera/depth/image_rect_raw /rscamera/color/image_raw /rscamera/infra1/image_rect_raw /rscamera/infra2/image_rect_raw /rscamera/color/camera_info /rscamera/depth/camera_info /rscamera/extrinsics/depth_to_color /rscamera/extrinsics/depth_to_infra1 /rscamera/extrinsics/depth_to_infra
  ```

- After recording the files I have used the [Python code](#Converting-bag-file-to-png-files) to convert the bag to png file and finally I choose about 10 pictures of them representing different poses of checkerboard.

- 

  

  # Converting bag file to png files

  I used the following script from [here](<https://gist.github.com/wngreene/835cda68ddd9c5416defce876a4d7dd9>).

  ```python
  #!/usr/bin/env python
  # -*- coding: utf-8 -*-
  
  # Copyright 2016 Massachusetts Institute of Technology
  
  """Extract images from a rosbag.
  """
  
  import os
  import argparse
  
  import cv2
  
  import rosbag
  from sensor_msgs.msg import Image
  from cv_bridge import CvBridge
  
  def main():
      """Extract a folder of images from a rosbag.
      """
      parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
      parser.add_argument("bag_file", help="Input ROS bag.")
      parser.add_argument("output_dir", help="Output directory.")
      parser.add_argument("image_topic", help="Image topic.")
  
      args = parser.parse_args()
  
      print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                            args.image_topic, args.output_dir)
  
      bag = rosbag.Bag(args.bag_file, "r")
      bridge = CvBridge()
      count = 0
      for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
          cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
          
          cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
          print "Wrote image %i" % count
  
          count += 1
  
      bag.close()
  
      return
  
  if __name__ == '__main__':
      main()
  ```

- and I run it with following command

  `python bagtoimg.py Checkboard.bag ./imgs/ /rscamera/color/image_raw`

- Then I selected about 10 of them.