#!/usr/bin/env python

"""
    Convert a skeleton transform tree to a list of visualization markers for RViz.
        
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('skeleton_markers')
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import Twist

"""
import roslib; roslib.load_manifest('rbx1_vision')
import rospy
import cv2
import cv2.cv as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
"""


class SkeletonMarkers():
    def __init__(self):
        rospy.init_node('markers_from_tf')
                
        rospy.loginfo("Initializing Skeleton Markers Node...")
        
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        
        # The tf_prefix needs to be set in the launch file for both
        # the openni_tracker node and the markers_from_tf node
        self.tf_prefix = rospy.get_param('~tf_prefix', '/skeleton')
        
        # There is usually no need to change the fixed frame from the default
        self.fixed_frame = rospy.get_param('~fixed_frame', 'openni_depth_frame')
        
        # We need to prepend the tf_prefix to the fixed frame
        self.fixed_frame = self.tf_prefix + '/' + self.fixed_frame

        # Initialize the tf listener
        tf_listener = tf.TransformListener()
        
        # Define a marker publisher
        marker_pub = rospy.Publisher('skeleton_markers', Marker)
        
        # Define a ROI publisher  -- BY zhanweelee
        self.roi_pub = rospy.Publisher('roi', RegionOfInterest)
        # Define a cmd_vel publisher  -- BY zhanweelee
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.move_cmd = Twist()
        
        # Intialize the markers
        self.initialize_markers()
        
        # Make sure we see the openni_depth_frame
        tf_listener.waitForTransform(self.fixed_frame, self.fixed_frame, rospy.Time(), rospy.Duration(60.0))
               
        # Begin the main loop
        while not rospy.is_shutdown():
            # Get the list of all skeleton frames from tf
            skeleton_frames = [f for f in tf_listener.getFrameStrings() if f.startswith(self.tf_prefix)]
            
            # If we don't have any frames yet, wait and try again
            if len(skeleton_frames) == 0:
                r.sleep()
                continue

            # Set the markers header
            self.markers.header.stamp = rospy.Time.now()
                        
            # Clear the markers point list
            self.markers.points = list()
            
            # Loop through the skeleton frames
            for frame in skeleton_frames:
                if frame == self.fixed_frame:
                    continue
                
                # Find the position of the frame's origin relative to the fixed frame.
                try:
                    position = Point()
                    
                    # Get the transformation from the fixed frame to the skeleton frame
                    (trans, rot)  = tf_listener.lookupTransform(self.fixed_frame, frame, rospy.Time(0))
                    position.x = trans[0]
                    position.y = trans[1]
                    position.z = trans[2]
                                                            
                    # Set a marker at the origin of this frame
                    self.markers.points.append(position)
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    rospy.logerr("tf error when looking up " + frame + ' and ' + self.fixed_frame)
                    continue
                
            
            # Publish the set of markers
            marker_pub.publish(self.markers)
            
            rospy.loginfo("self.tf_prefix: ")
            rospy.loginfo(self.tf_prefix)

            # When using ROI in practice we've faced some practical problems. It caused the low accuracy of operation.
            # So we decided to directly use TF data to control the robot.
            # We make the change directly on the file of making markers.
            # The code we design is below:
            (trans, rot)  = tf_listener.lookupTransform(self.fixed_frame, self.tf_prefix + "/right_hand_1", rospy.Time(0))
            ROI = RegionOfInterest()
            if trans[2] < 0:              # Decide the direction of angular speed.
                dz = -1
            else:
                dz = 1
            tmpy = abs(trans[2])
            if tmpy < 0.1:                # If the change is too small, then we regard it as no change.
                tmpy = 0
            if trans[1] < 0:              # Decide the direction of linear speed.
                dy = -1
            else:
                dy = 1
            tmpz = abs(trans[1])
            if tmpz < 0.1:                # If the change is too small, then we regard it as no change.
                tmpz = 0
            self.move_cmd.angular.z = -dz * (0.1 + 0.8 * tmpy)    # I design this formular to give both speed a bias equal to 0.1 and the variance between
                                                                  # 0 to 0.8. Because I'm afraid of breaking the turtlebot, so I set the speed very slow. If
                                                                  # needed, I can change the bias and the coefficient to get better performence.   --By Leo
            self.move_cmd.linear.x = -dy * (0.1 + 0.8 * tmpz)
            self.cmd_vel_pub.publish(self.move_cmd)
                                   
            r.sleep()
            
    def initialize_markers(self):
        # Set various parameters
        scale = rospy.get_param('~scale', 0.07)
        lifetime = rospy.get_param('~lifetime', 0) # 0 is forever
        ns = rospy.get_param('~ns', 'skeleton_markers')
        id = rospy.get_param('~id', 0)
        color = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})
        
        # Initialize the marker points list
        self.markers = Marker()
        self.markers.header.frame_id = self.fixed_frame
        self.markers.ns = ns
        self.markers.id = id
        self.markers.type = Marker.POINTS
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(lifetime)
        self.markers.scale.x = scale
        self.markers.scale.y = scale
        self.markers.color.r = color['r']
        self.markers.color.g = color['g']
        self.markers.color.b = color['b']
        self.markers.color.a = color['a']
        
if __name__ == '__main__':
    try:
        SkeletonMarkers()
    except rospy.ROSInterruptException:
        pass
