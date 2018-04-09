#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

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
    msg.color.r = 253./255.
    msg.color.g = 106./255.
    msg.color.b = 2./255.
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

    # Setup tf Publishers
    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.Subscriber('/husky_velocity_controller/odom',Odometry,odometryCb)
    rospy.Subscriber('/vive/LHR_0EB0243A_odom',Odometry,viveCb)

    # Draw Landmarks
    landmarks = rospy.get_param('landmarks')
    global landmark_viz_pub, landmark_id
    landmark_viz_pub = rospy.Publisher('landmark_viz', Marker, queue_size=100)
    landmark_id = 0

    rospy.sleep(0.1) # delay for markers
    for i in range(0,len(landmarks)):
        x = landmarks[i]['pos'][0]
        y = landmarks[i]['pos'][1]
        z = 0
        size = landmarks[i]['size']
        if size == 'small':
            landmarker(x,y,z,0.2)
        else:
            landmarker(x,y,z,0.35)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        # try:
        #     (vive_trans,vive_rot) = listener.lookupTransform('/odom', '/vive', rospy.Time(0))
        #     print "vive:",vive_trans
        #
        #     (odom_trans,odom_rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        #     print "odom:",odom_trans
        #
        #     print "delta:",np.array(vive_trans) - np.array(odom_trans)
        #
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

        r.sleep()

if __name__ == "__main__":
    main()
