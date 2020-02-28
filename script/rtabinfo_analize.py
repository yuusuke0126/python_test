#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, REC Robotics Equipment Corporation GmbH, Planegg
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

import sys
import json

from math import sin, cos, pi, sqrt
import numpy

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rtabmap_ros.msg import Info
from std_msgs.msg import Float32

class rtabmapAnalizer():
  def __init__(self):
    rospy.init_node('rtab_info_analize_node', anonymous=True)
    rospy.Subscriber("/rtabmap/info", Info, self.callback)
    self.pub = rospy.Publisher("/localization_certainty", Float32, queue_size=1)

  def callback(self, data):
    lin_var = data.statsValues[data.statsKeys.index('Loop/Linear_variance/')]
    ang_var = data.statsValues[data.statsKeys.index('Loop/Angular_variance/')]
    if lin_var == 0.0:
      localization_certainty = 0.0
    else:
      localization_certainty = 1.0 / lin_var
    rospy.loginfo(lin_var - ang_var)
    self.pub.publish(localization_certainty/300.0)
    

    # self.last_time = rospy.Time.now()
  
  def startDrive(self):
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      # rospy.loginfo('test')
      rate.sleep()
      

if __name__ == '__main__':
  # myargv = rospy.myargv(argv=sys.argv)
  # if len(myargv)>1:
  #   URL = URL.replace("127.0.0.1",myargv[1])
  # print("connecting to: ",URL)
  try:
    ra = rtabmapAnalizer()
    ra.startDrive()
  except rospy.ROSInterruptException:
    pass
