#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray
import tf
import math
import pdb

# def plot_point(self,x,y,z,col):
#     # Visualize probe point
#     global contact_viz_id
#     global contact_viz_pub
#     msg = Marker()
#     msg.header.frame_id = "gantry"
#     # msg.header.frame_id = "probe_tip" # DEBUG
#     msg.header.seq = self.contact_viz_id
#     msg.header.stamp = rospy.Time.now()
#     msg.ns = "probe_contact_viz"
#     msg.id = self.contact_viz_id
#     msg.type = 2  # cube
#     msg.action = 0  # add
#     msg.pose.position = Point(x,y,z)
#     msg.pose.orientation.w = 1
#     msg.scale.x = 10
#     msg.scale.y = 10
#     msg.scale.z = 10
#     msg.color.a = 1.0
#     msg.color.r = col[0]
#     msg.color.g = col[1]
#     msg.color.b = col[2]
#     self.contact_viz_pub.publish(msg)
#     self.contact_viz_id += 1

def main():

    rospy.init_node('ground_truth')

    br = tf.TransformBroadcaster()

    loc = rospy.get_param('loc')

    print loc

    r = rospy.Rate(50) # 50 Hz
    while not rospy.is_shutdown():

        br.sendTransform((0,0,0),
            tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),
            "testing",
            "world")

        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
