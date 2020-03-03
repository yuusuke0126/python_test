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

import numpy as np
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from python_test.msg import PositionArray


class GlobalpathAnalizer():
  def __init__(self):
    rospy.init_node('globalpath_analizer', anonymous=True)
    rospy.Subscriber("/move_base/PipelinePlanner/plan", Path, self.callback)
    self.pub = rospy.Publisher("/global_plan", PositionArray, queue_size=1)

  def callback(self, data):
    dt = 0.2
    vx = 0.4
    i = 0
    pos = np.zeros((2,len(data.poses)))
    for posestamped in data.poses:
      pos[:,i:i+1] = np.array([[posestamped.pose.position.x],[posestamped.pose.position.y]])
      i += 1
    pos_diff = pos[:,1:] - pos[:,:-1]
    diff_norm = np.linalg.norm(pos_diff, axis=0)
    dist = np.dot(diff_norm, np.triu(np.ones(np.diag(diff_norm).shape)))
    dist = np.insert(dist, 0, 0.0)
    i = 0
    dist_index = []
    for t in np.arange(0,dist[-1]/vx, dt):
      for j in range(i,dist.shape[0]):
        if dist[j] >= vx * t:
          dist_index.append(j)
          i = j + 1
          break
    
    position_msg = PositionArray()
    position_msg.header.stamp = rospy.Time.now()
    rospy.loginfo("test")
    for i in dist_index:
      position = Point(); position.x = pos[0,i]; position.y = pos[1,i]
      position_msg.positions.append(position)
    # rospy.loginfo(pos[:,dist_index])
    self.pub.publish(position_msg)
  
  def startDrive(self):
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      # rospy.loginfo('test')
      rate.sleep()
      

if __name__ == '__main__':
  try:
    ga = GlobalpathAnalizer()
    ga.startDrive()
  except rospy.ROSInterruptException:
    pass
