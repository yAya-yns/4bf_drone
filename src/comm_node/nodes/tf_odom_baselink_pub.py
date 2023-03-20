#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import PoseStamped
import tf

def pub_pose_cb(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "camera_odom_frame")
    print(msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('tf_odom_baselink_pub')
    local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = pub_pose_cb)
    rospy.spin()