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
Visualize route plan for geographic map.
"""

PKG_NAME = 'route_network'
import roslib
roslib.load_manifest(PKG_NAME)
import rospy

import sys
import random
import geodesy.wu_point

from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RouteSegment, RoutePath
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

try:
    from geographic_msgs.msg import UniqueID
except ImportError:
    from uuid_msgs.msg import UniqueID


class VizPlanNode():

    def __init__(self):
        """ROS node to visualize a route plan.  """
        rospy.init_node('viz_route_plan')
        self.graph = None

        # advertise visualization marker topic
        self.pub = rospy.Publisher('visualization_marker_array',
                                   MarkerArray, latch=True)

        # subscribe to route network
        self.sub = rospy.Subscriber('route_network', RouteNetwork,
                                    self.graph_callback)
        self.plan_sub = rospy.Subscriber('route_plan', RoutePath,
                                    self.plan_callback)

        self.timer_interval = rospy.Duration(4)
        self.marker_life = self.timer_interval + rospy.Duration(1)
        # rospy.Timer(self.timer_interval, self.timer_callback)

    def graph_callback(self, graph):
        """Handle RouteNetwork message.

        :param graph: RouteNetwork message.

        :post: self.graph = RouteNetwork message
        :post: self.points = visualization markers message.
        """
        self.points = geodesy.wu_point.WuPointSet(graph.points)
        self.segment_ids = {}   # segments symbol table
        for sid in xrange(len(graph.segments)):
            self.segment_ids[graph.segments[sid].id.uuid] = sid

        self.graph = graph

    def plan_callback(self, msg):
        if self.graph:
            self.mark_plan(msg)
        else:
            rospy.logwarn('cannot draw plan, have not received a route network graph yet!')

    def mark_plan(self, plan):
        """Publish visualization markers for a RoutePath.

        :param plan: RoutePath message
        """
        marks = MarkerArray()
        hdr = self.graph.header
        hdr.stamp = rospy.Time.now()
        index = 0
        for seg_msg in plan.segments:
            marker = Marker(header=hdr,
                            ns='plan_segments',
                            id=index,
                            type=Marker.LINE_STRIP,
                            action=Marker.ADD,
                            scale=Vector3(x=4.0),
                            color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8),
                            lifetime=self.marker_life)
            index += 1
            segment = self.graph.segments[self.segment_ids[seg_msg.uuid]]
            marker.points.append(self.points[segment.start.uuid].toPointXY())
            marker.points.append(self.points[segment.end.uuid].toPointXY())
            marks.markers.append(marker)

        self.pub.publish(marks)


def main():
    node_class = VizPlanNode()
    try:
        rospy.spin()            # wait for messages
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
