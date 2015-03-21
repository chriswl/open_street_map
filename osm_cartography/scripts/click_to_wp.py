#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
convert the coordinates of a point clicked on in rviz to utm coordinates and
return the closest waypoint/osm marker
"""

from __future__ import print_function

PKG_NAME = 'osm_cartography'
import roslib
roslib.load_manifest(PKG_NAME)
import rospy

import sys
import itertools
import geodesy.props
import geodesy.utm
import geodesy.wu_point
from geodesy import bounding_box

from geographic_msgs.msg import GeoPoint
from geographic_msgs.srv import GetGeographicMap
from geometry_msgs.msg import PointStamped, Point

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

import tf
import numpy as np

# dynamic parameter reconfiguration
from dynamic_reconfigure.server import Server as ReconfigureServer
import osm_cartography.cfg.VizOSMConfig as Config


class ClickNode():

    def __init__(self):
        """ROS node to publish visualization markers for a GeographicMap.
        """
        rospy.init_node('click_to_wp')

        # get map
        self.map = None
        self.msg = None
        rospy.wait_for_service('get_geographic_map')
        self.get_map = rospy.ServiceProxy('get_geographic_map',
                                          GetGeographicMap)

        self.sub = rospy.Subscriber('/clicked_point',
                                    PointStamped,
                                    self.click_cb)

        self.listener = tf.TransformListener()

        # advertise visualization marker topic
        self.pub = rospy.Publisher('visualization_marker',
                                   Marker, queue_size=10)

        try:
            resp = self.get_map('', None)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed:", str(e))
            # ignore new config, it failed
        else:                           # get_map returned
            if resp.success:
                self.gmap = resp.map
                self.map_points = geodesy.wu_point.WuPointSet(self.gmap.points)
            else:
                print('get_geographic_map failed, status:', str(resp.status))

        self.marker = None

        self.timer_interval = rospy.Duration(1)
        rospy.Timer(self.timer_interval, self.pub_marker)

    def click_cb(self, msg):
        # rospy.loginfo('I heard a click: {}'.format(msg))
        wp = self.find_closest_wp(msg)
        self.mark_wp(wp)

    def find_closest_wp(self, msg):
        map_point = self.listener.transformPoint('/map', msg)
        all_points = np.zeros((len(self.map_points), 2))
        point_ids = []
        for i, wp in enumerate(self.map_points):
            pt = wp.utm.toPoint()
            all_points[i] = [pt.x, pt.y]
            point_ids.append(wp.uuid())

        dists = np.linalg.norm(all_points - np.array([map_point.point.x, map_point.point.y]), axis=1)
        wp = self.map_points[point_ids[np.argmin(dists)]]
        print(wp)
        return wp

    def mark_wp(self, wp):
        color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.8)
        marker = Marker(  # header=self.gmap.header,
            ns="marked_wp",
            id=500000,
            type=Marker.CUBE,
            action=Marker.ADD,
            scale=Vector3(x=41, y=41, z=41),
            color=color,
            lifetime=rospy.Duration(10))
        # use easting and northing coordinates (ignoring altitude)
        marker.pose.position = wp.toPointXY()
        marker.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time.now()

        self.marker = marker

    def pub_marker(self, event):
        if self.marker:
            self.pub.publish(self.marker)


def main():
    node = ClickNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
