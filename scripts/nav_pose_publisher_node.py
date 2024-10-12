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


import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import numpy as np
import math
import time
import sys
import tf
import yaml

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_nav

from std_msgs.msg import Bool, String, Float32, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.msg import NavPose, NavPosePub
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest
from nepi_ros_interfaces.srv import NavPosePubQuery, NavPosePubQueryResponse

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

#########################################
# Node Class
#########################################


class NavPosePublisher(object):

  NAVPOSE_PUB_RATE_OPTIONS = [1.0,2.0] 
  NAVPOSE_3D_FRAME_OPTIONS = ['ENU','NED']
  NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']

  FACTORY_PUB_RATE_HZ = 1
  FACTORY_3D_FRAME = 'ENU'
  FACTORY_ALT_FRAME = 'WGS84'

  data_products = ['navpose']
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "nav_pose_publisher_app" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################


    ## Initialize Class Variables
    self.last_navpose = None
    ## Define Class Services Messages
    self.navpose_pub_options_report = NavPosePubQueryResponse()
    self.navpose_pub_options_report.pub_rate_min_max = self.NAVPOSE_PUB_RATE_OPTIONS
    self.navpose_pub_options_report.frame_3d_options = self.NAVPOSE_3D_FRAME_OPTIONS
    self.navpose_pub_options_report.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
    rospy.Service('~navpose_pub_options_query', NavPosePubQuery, self.provide_navpose_capabilities)
    # NavPose Heading, Orientation, Location, and Position Publish Topics
    ## Define Class Services Calls
    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    self.NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
    nepi_msg.publishMsgInfo(self,"looking for nav_pose service at " + self.NAVPOSE_SERVICE_NAME)
    rospy.wait_for_service(self.NAVPOSE_SERVICE_NAME)
    nepi_msg.publishMsgInfo(self,"found nav_pose service")
 
    ## Create Class Publishers
    self.navpose_pub = rospy.Publisher("~navpose", NavPosePub, queue_size=1, latch = True)

    ## Start Class Subscribers
    rospy.Subscriber('~set_pub_rate', Float32, self.setPublishRateCb, queue_size=1) # start local callback
    rospy.Subscriber('~set_3d_frame', String, self.set3dFrameCb, queue_size=1) # start local callback
    rospy.Subscriber('~set_alt_frame', String, self.setAltFrameCb, queue_size=1) # start local callback
    ## Set up save and config interfaces

    factory_data_rates = {}
    for d in self.data_products:
        factory_data_rates[d] = [1.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
    self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)
    # Temp Fix until added as NEPI ROS Node
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)

    # Start navpose data publishers
    nepi_ros.timer(nepi_ros.duration(1), self.navpose_get_publish_callback, oneshot = True)

    ## Initialize From Param Server
    self.initParamServerValues(do_updates = True)
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
    nepi_ros.spin()

  #######################
  ### Node Methods

  def provide_navpose_capabilities(self, _):
      self.navpose_pub_options_report.set_pub_rate = nepi_ros.get_param(self,"~pub_rate",self.init_pub_rate)
      self.navpose_pub_options_report.set_3d_frame = nepi_ros.get_param(self,"~frame_3d",self.init_3d_frame)
      self.navpose_pub_options_report.set_alt_frame = nepi_ros.get_param(self,"~frame_alt",self.init_alt_frame)
      return self.navpose_pub_options_report 

  def setPublishRateCb(self,msg):
    rate = msg.data
    min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
    max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
    if rate < min:
      rate = min
    if rate > max:
      rate = max
    nepi_ros.set_param(self,"~pub_rate",rate)

  def set3dFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_3D_FRAME_OPTIONS:
      nepi_ros.set_param(self,"~frame_3d",frame)

  def setAltFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_ALT_FRAME_OPTIONS:
      nepi_ros.set_param(self,"~frame_alt",frame)

  def setCurrentAsDefault(self):
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #nepi_msg.publishMsgWarn(self,"Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
      nepi_msg.publishMsgInfo(self,"Setting init values to param values")
      self.init_pub_rate = nepi_ros.get_param(self,"~pub_rate",self.FACTORY_PUB_RATE_HZ)
      self.init_3d_frame = nepi_ros.get_param(self,"~frame_3d",self.FACTORY_3D_FRAME)
      self.init_alt_frame = nepi_ros.get_param(self,"~frame_alt",self.FACTORY_ALT_FRAME)
      self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
      nepi_ros.set_param(self,"~pub_rate",self.init_pub_rate)
      nepi_ros.set_param(self,"~frame_3d",self.init_3d_frame)
      nepi_ros.set_param(self,"~frame_alt",self.init_alt_frame)
      if do_updates:
          self.updateFromParamServer()


  ### Setup a regular background navpose get and publish timer callback
  def navpose_get_publish_callback(self,timer):
    ros_timestamp = nepi_ros.time_now()
    set_pub_rate = nepi_ros.get_param(self,"~pub_rate",self.init_pub_rate)
    set_3d_frame = nepi_ros.get_param(self,"~frame_3d",self.init_3d_frame)
    set_alt_frame = nepi_ros.get_param(self,"~frame_alt",self.init_alt_frame)
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    current_navpose = None
    try:
      nav_pose_response = None
      get_navpose_service = rospy.ServiceProxy(self.NAVPOSE_SERVICE_NAME, NavPoseQuery)
      nav_pose_response = get_navpose_service(NavPoseQueryRequest())
      current_navpose = nav_pose_response.nav_pose
    except rospy.ServiceException as e:
      nepi_msg.publishMsgInfo(self,"Service call failed: " + str(e))
      time.sleep(1)

    if current_navpose != None:
      if self.last_navpose != current_navpose:
        # Get current navpose
        # Get current heading in degrees
        current_heading_deg = nepi_nav.get_navpose_heading_deg(nav_pose_response)
        # Get current orientation vector (roll, pitch, yaw) in degrees enu frame
        if set_3d_frame == 'NED':
          # Get current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
          ort_values = nepi_nav.get_navpose_orientation_ned_degs(nav_pose_response)
          current_orientation_degs = Vector3()
          current_orientation_degs.x = ort_values[0]
          current_orientation_degs.y = ort_values[1]
          current_orientation_degs.z = ort_values[2]

          # Get current position vector (x, y, z) in meters ned frame
          pos_values = nepi_nav.get_navpose_position_ned_m(nav_pose_response)
          current_position_m = Vector3()
          current_position_m.x = pos_values[0]
          current_position_m.y = pos_values[1]
          current_position_m.z = pos_values[2]
        else:
          ort_values = nepi_nav.get_navpose_orientation_enu_degs(nav_pose_response)
          current_orientation_degs = Vector3()
          current_orientation_degs.x = ort_values[0]
          current_orientation_degs.y = ort_values[1]
          current_orientation_degs.z = ort_values[2]
          # Get current position vector (x, y, z) in meters enu frame
          pos_values = nepi_nav.get_navpose_position_enu_m(nav_pose_response)
          current_position_m = Vector3()
          current_position_m.x = pos_values[0]
          current_position_m.y = pos_values[1]
          current_position_m.z = pos_values[2]

        if set_alt_frame == "AMSL":
          # Get current location vector (lat, long, alt) in geopoint data with AMSL height
          geo_values =  nepi_nav.get_navpose_location_amsl_geo(nav_pose_response)
          current_location_geo = Vector3()
          current_location_geo.x = geo_values[0]
          current_location_geo.y = geo_values[1]
          current_location_geo.z = geo_values[2]
        else:
          # Get current location vector (lat, long, alt) in geopoint data with WGS84 height
          geo_values =  nepi_nav.get_navpose_location_wgs84_geo(nav_pose_response)  
          current_location_geo = Vector3()
          current_location_geo.x = geo_values[0]
          current_location_geo.y = geo_values[1]
          current_location_geo.z = geo_values[2]
        # Get current geoid heihgt
        current_geoid_height =  nepi_nav.get_navpose_geoid_height(nav_pose_response)
        # Publish new current navpose data
        current_navpose = NavPosePub()
        current_navpose.header.stamp = ros_timestamp
        current_navpose.set_pub_rate = set_pub_rate
        current_navpose.set_3d_frame = set_3d_frame
        current_navpose.set_alt_frame = set_alt_frame
        current_navpose.geoid_height_meters = current_geoid_height
        current_navpose.heading_deg = current_heading_deg
        current_navpose.orientation_rpy_degs = current_orientation_degs
        current_navpose.position_xyz_meters = current_position_m
        current_navpose.location_geo = current_location_geo

        if not rospy.is_shutdown():
          self.navpose_pub.publish(current_navpose)


        np_dict = dict()
        np_dict['timestamp'] = nepi_ros.get_datetime_dict_from_stamp(ros_timestamp)
        np_dict['3d_frame'] = set_3d_frame
        np_dict['alt_frame'] = set_alt_frame
        np_dict['geoid_height_meters'] = current_geoid_height
        np_dict['heading_deg'] = current_heading_deg
        np_dict['orientation_rpy_degs'] = {'roll_deg': float(ort_values[0]),'pitch_deg': float(ort_values[1]),'yaw_deg': float(ort_values[2])}
        np_dict['position_xyz_meters'] =  {'x_m': float(pos_values[0]),'y_m': float(pos_values[1]),'z_m': float(pos_values[2])}
        np_dict['location_geo'] =  {'latitude': float(geo_values[0]),'longitude': float(geo_values[1]),'altitude_m': float(geo_values[2])}
        #nepi_msg.publishMsgWarn(self,"New nav_pose entry: " + str(np_dict))
        self.save_dict2file(self.node_name,'navpose',np_dict,ros_timestamp)

    # Setup nex update check
    self.last_navpose = current_navpose
    rate = set_pub_rate
    sleep_time = float(1.0)/rate - 1
    if sleep_time < 0:
      sleep_time = 0
    nepi_ros.sleep(sleep_time)
    nepi_ros.timer(nepi_ros.duration(1), self.navpose_get_publish_callback, oneshot = True)


  def save_dict2file(self,node_name,data_product,data_dict,ros_timestamp):
      if self.save_data_if is not None:
        saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
        if data_dict is not None:
            if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                        node_name + "_" + data_product, 'yaml')
                if os.path.isfile(full_path_filename) is False:
                    with open(full_path_filename, 'w') as f:
                        yaml.dump(data_dict, f)
                self.save_data_if.data_product_snapshot_reset(data_product)

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPosePublisher()


