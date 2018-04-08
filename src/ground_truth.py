#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray
from visualization_msgs.msg import Marker

import pdb


def landmarker(x,y,z,size):
    global landmark_id
    global landmark_viz_pub
    msg = Marker()
    msg.header.frame_id = "world"
    msg.header.seq = landmark_id
    msg.header.stamp = rospy.Time.now()
    msg.ns = "landmark"
    msg.id = landmark_id
    msg.type = 2  # cube
    msg.action = 0  # add
    msg.pose.position = Point(x,y,z)
    msg.pose.orientation.w = 1
    msg.scale.x = size
    msg.scale.y = size
    msg.scale.z = size
    msg.color.a = 1.0
    msg.color.r = 255
    msg.color.g = 0
    msg.color.b = 0
    landmark_viz_pub.publish(msg)
    landmark_id += 1

def main():

    rospy.init_node('ground_truth')

    landmarks = rospy.get_param('landmarks')
    vive_base_1 = rospy.get_param('vive_base_1')
    vive_base_2 = rospy.get_param('vive_base_2') # also vive_world apparently
    odom_pos = rospy.get_param('odom_pos')
    odom_rot = rospy.get_param('odom_rot')

    br = tf.TransformBroadcaster()

    global landmark_viz_pub, landmark_id
    landmark_viz_pub = rospy.Publisher('landmark_viz', Marker, queue_size=100)
    landmark_id = 0

    rospy.sleep(0.3)
    for i in range(0,len(landmarks)):
        x = landmarks[i]['pos'][0]
        y = landmarks[i]['pos'][1]
        z = 0
        size = landmarks[i]['size']
        landmarker(x,y,z,0.2)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/world', '/odom', rospy.Time(0))
            print trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue




        ### This is basically a hacaky way to paramterize a static tf ###

        br.sendTransform((vive_base_1[0],vive_base_1[1],0),
            tf.transformations.quaternion_from_euler(1.5708,0,0),
            rospy.Time.now(),
            "/vive_base_1",
            "/world")

        br.sendTransform((vive_base_2[0],vive_base_2[1],0),
            tf.transformations.quaternion_from_euler(1.5708,0,0),
            rospy.Time.now(),
            "/vive_base_2",
            "/world")

        br.sendTransform((vive_base_2[0],vive_base_2[1],0),
            tf.transformations.quaternion_from_euler(1.5708,0,1.5708),
            rospy.Time.now(),
            "/vive_world",
            "/world") # updating static tf

        br.sendTransform((odom_pos[0],odom_pos[1],odom_pos[2]),
            tf.transformations.quaternion_from_euler(odom_rot[0],odom_rot[1],odom_rot[2]),
            rospy.Time.now(),
            "/odom",
            "/world") # updating static tf

        r.sleep()

if __name__ == "__main__":
    main()
