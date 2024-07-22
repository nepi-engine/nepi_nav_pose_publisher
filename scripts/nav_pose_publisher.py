#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script.
# 1. Calls the NEPI ROS's nav_pose_mgr/query_data_products service
# 2. Gets and publishes current navpose solution data at set rate to new topics:
#  navpose = NEPI NavPose Message
#  heading_deg = Float32 (heading in degrees)
#  orientation_ned_degs = [Float32, Float32, Float32] (roll, pitch, yaw in +-180 degrees NED frame)  
#  orientation_enu_degs = [Float32, Float32, Float32] (roll, pitch, yaw in +-180 degrees ENU frame)  
#  position_ned_m = [Float32, Float32, Float32] (x, y, z in meters NED frame)  
#  position_enu_m = [Float32, Float32, Float32] (x, y, z in meters ENU frame)  
#  location_amsl_geo = [Float32, Float32, Float32] (lat, long, altitude in meters AMSL height)
#  location_wgs84_geo = [Float32, Float32, Float32] (lat, long, altitude in meters WGS-84 Ellipoid height)
#  geoid_height_m = Float32 (meters geoid height added to AMSL height to convert to WGS84 height)
#
# Orientation and Position data are published in both ROS standard ENU reference frame
# and a robot standard NED reference frame for convenience. Learn more about these frames and converting them at:
# https://github.com/mavlink/mavros/issues/216
#
# Location Geo Altitudes are published in both meters above mean sea level (AMSL)and meters above the WGS-84 Ellipsoid (WGS84)


import rospy
import numpy as np
import math
import time
import sys
import tf
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_nav

from std_msgs.msg import Bool, String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.msg import NavPose
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest

#########################################
# Node Class
#########################################

class NavPosePublisher(object):

  #######################
  ### Node Initialization
  NAVPOSE_PUB_RATE_HZ = 10
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")
    ## Initialize Class Variables
    self.last_navpose = None
    ## Define Class Namespaces
    # NavPose Heading, Orientation, Location, and Position Publish Topics
    ## Define Class Services Calls
    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    self.NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
    rospy.loginfo("Nav Pose Publisher looking for nav_pose service at " + self.NAVPOSE_SERVICE_NAME)
    rospy.wait_for_service(self.NAVPOSE_SERVICE_NAME)
    rospy.loginfo("Nav Pose Publisher found nav_pose service")
    ## Create Class Sevices    
    ## Create Class Publishers
    self.navpose_navpose_pub = rospy.Publisher("~navpose", NavPose, queue_size=1, latch = True)
    self.navpose_heading_pub = rospy.Publisher("~heading_deg", Float32, queue_size=1, latch = True)
    self.navpose_orientation_ned_pub = rospy.Publisher("~orientation_ned_degs", Vector3 , queue_size=1, latch = True)
    self.navpose_orientation_enu_pub = rospy.Publisher("~orientation_enu_degs", Vector3 , queue_size=1, latch = True)
    self.navpose_position_ned_pub = rospy.Publisher("~position_ned_m", Vector3 , queue_size=1, latch = True)
    self.navpose_position_enu_pub = rospy.Publisher("~position_enu_m", Vector3 , queue_size=1, latch = True)
    self.navpose_location_amsl_pub = rospy.Publisher("~location_amsl_geo", Vector3 , queue_size=1, latch = True)
    self.navpose_location_wgs84_pub = rospy.Publisher("~location_wgs84_geo", Vector3 , queue_size=1, latch = True)
    self.navpose_geoid_height_pub = rospy.Publisher("~geoid_height_m", Float32, queue_size=1, latch = True)
    self.navpose_pub_interval_sec = float(1.0)/self.NAVPOSE_PUB_RATE_HZ
    ## Start Class Subscribers
    ## Start Node Processes

    # Start navpose data publishers
    rospy.Timer(rospy.Duration(self.navpose_pub_interval_sec), self.navpose_get_publish_callback)
    ## Initiation Complete
    rospy.loginfo("Initialization Complete")

  #######################
  ### Node Methods

  ### Setup a regular background navpose get and publish timer callback
  def navpose_get_publish_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    current_navpose = None
    try:
      nav_pose_response = None
      get_navpose_service = rospy.ServiceProxy(self.NAVPOSE_SERVICE_NAME, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      #rospy.loginfo(nav_pose_response)
      current_navpose = nav_pose_response.nav_pose
    except rospy.ServiceException as e:
      rospy.loginfo("Service call failed: " + str(e))
      time.sleep(1)

    if current_navpose != None:
      if self.last_navpose == None:
        pub_new_data = True
      else:
        #rospy.loginfo(current_navpose)
        pub_new_data = (self.last_navpose.fix != current_navpose.fix or self.last_navpose.odom \
              != current_navpose.odom or self.last_navpose.heading != current_navpose.heading  )
      if pub_new_data:
          # Get current navpose
        # Get current heading in degrees
        current_heading_deg = nepi_nav.get_navpose_heading_deg(nav_pose_response)
        # Get current orientation vector (roll, pitch, yaw) in degrees enu frame

        values = nepi_nav.get_navpose_orientation_enu_degs(nav_pose_response)
        current_orientation_enu_degs = Vector3()
        current_orientation_enu_degs.x = values[0]
        current_orientation_enu_degs.y = values[1]
        current_orientation_enu_degs.y = values[2]
        # Get current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
        values = nepi_nav.get_navpose_orientation_ned_degs(nav_pose_response)
        current_orientation_ned_degs = Vector3()
        current_orientation_ned_degs.x = values[0]
        current_orientation_ned_degs.y = values[1]
        current_orientation_ned_degs.y = values[2]
        # Get current position vector (x, y, z) in meters enu frame
        values = nepi_nav.get_navpose_position_enu_m(nav_pose_response)
        current_position_enu_m = Vector3()
        current_position_enu_m.x = values[0]
        current_position_enu_m.y = values[1]
        current_position_enu_m.y = values[2]
        # Get current position vector (x, y, z) in meters ned frame
        values = nepi_nav.get_navpose_position_ned_m(nav_pose_response)
        current_position_ned_m = Vector3()
        current_position_ned_m.x = values[0]
        current_position_ned_m.y = values[1]
        current_position_ned_m.y = values[2]
        # Get current location vector (lat, long, alt) in geopoint data with WGS84 height
        values =  nepi_nav.get_navpose_location_wgs84_geo(nav_pose_response)  
        current_location_wgs84_geo = Vector3()
        current_location_wgs84_geo.x = values[0]
        current_location_wgs84_geo.y = values[1]
        current_location_wgs84_geo.y = values[2]
        # Get current location vector (lat, long, alt) in geopoint data with AMSL height
        values =  nepi_nav.get_navpose_location_amsl_geo(nav_pose_response)
        current_location_amsl_geo = Vector3()
        current_location_amsl_geo.x = values[0]
        current_location_amsl_geo.y = values[1]
        current_location_amsl_geo.y = values[2]
        # Get current geoid heihgt
        current_geoid_height =  nepi_nav.get_navpose_geoid_height(nav_pose_response)
        # Publish new current navpose data
        self.navpose_navpose_pub.publish(current_navpose)
        self.navpose_heading_pub.publish(current_heading_deg)
        self.navpose_orientation_ned_pub.publish(current_orientation_ned_degs)
        self.navpose_orientation_enu_pub.publish(current_orientation_enu_degs)
        self.navpose_position_ned_pub.publish(current_position_ned_m)
        self.navpose_position_enu_pub.publish(current_position_enu_m)
        self.navpose_location_amsl_pub.publish(current_location_amsl_geo)
        self.navpose_location_wgs84_pub.publish(current_location_wgs84_geo)
        self.navpose_geoid_height_pub.publish(current_geoid_height)

        self.last_navpose = current_navpose

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
def main():
    rospy.init_node("nav_pose_publisher")
    node = NavPosePublisher()
    rospy.spin()

if __name__ == '__main__':
    main()


