#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16, Int16MultiArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

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
    msg.type = 2  # sphere
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


def odometryCb(msg):

    p = msg.pose.pose.position
    r = msg.pose.pose.orientation

    br.sendTransform((p.x,p.y,p.z),
        (r.x,r.y,r.z,r.w),
        rospy.Time.now(),
        "/base_link",
        "/odom")

def viveCb(msg):

    # let's do some mad dog hacking to normalize this
    p = msg.pose.pose.position
    r = msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((r.x,r.y,r.z,r.w))

    if abs(yaw) > math.pi/2:
        new_yaw = -1*pitch
    else:
        new_yaw = 1*pitch + math.pi

    br.sendTransform((p.x,0,p.z),
        tf.transformations.quaternion_from_euler(-math.pi/2,new_yaw,0),
        rospy.Time.now(),
        "/vive",
        "/vive_world")


def main():

    rospy.init_node('ground_truth')
    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    landmarks = rospy.get_param('landmarks')
    vive_base_1 = rospy.get_param('vive_base_1')
    vive_base_2 = rospy.get_param('vive_base_2') # also vive_world apparently
    odom_pos = rospy.get_param('odom_pos')
    odom_rot = rospy.get_param('odom_rot')
    velodyne_offset = rospy.get_param('velodyne_offset')

    global landmark_viz_pub, landmark_id
    landmark_viz_pub = rospy.Publisher('landmark_viz', Marker, queue_size=100)
    landmark_id = 0

    rospy.Subscriber('/husky_velocity_controller/odom',Odometry,odometryCb)
    rospy.Subscriber('/vive/LHR_0EB0243A_odom',Odometry,viveCb)

    rospy.sleep(0.3)
    for i in range(0,len(landmarks)):
        x = landmarks[i]['pos'][0]
        y = landmarks[i]['pos'][1]
        z = 0
        size = landmarks[i]['size']
        landmarker(x,y,z,0.2)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        ### This is basically a hacky way to paramterize a static tf ###

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
            "/world")

        br.sendTransform((odom_pos[0],odom_pos[1],odom_pos[2]),
            tf.transformations.quaternion_from_euler(odom_rot[0],odom_rot[1],odom_rot[2]),
            rospy.Time.now(),
            "/odom",
            "/world")

        br.sendTransform((velodyne_offset[0],velodyne_offset[1],velodyne_offset[2]),
            tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),
            "/laser_link",
            "/base_link")

        r.sleep()

if __name__ == "__main__":
    main()
