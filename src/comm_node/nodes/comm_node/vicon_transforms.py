#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, PoseArray
from tf.transformations import quaternion_from_matrix

'''
  Create Vicon to Odom Transformation
  @param_in vicon: drone to Vicon TransformStamped
  @param_in odom: drone to odom PoseStamped
  @returns Vicon To Odom matrix
'''
def get_vicon_to_odom_transform(vicon, odom):
    # create 4x4 vicon transform matrix:
    q0 = vicon.transform.rotation.w
    q1 = vicon.transform.rotation.x
    q2 = vicon.transform.rotation.y
    q3 = vicon.transform.rotation.z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    t_x = vicon.transform.translation.x
    t_y = vicon.transform.translation.y
    t_z = vicon.transform.translation.z
     
    # 4x4 rotation matrix
    mat_drone_vicon = np.array([[r00, r01, r02, t_x],
                                [r10, r11, r12, t_y],
                                [r20, r21, r22, t_z],
                                [0,   0,   0,   1]])

    # create 4x4 odom transform matrix:
    q0 = odom.pose.orientation.w
    q1 = odom.pose.orientation.x
    q2 = odom.pose.orientation.y
    q3 = odom.pose.orientation.z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    t_x = odom.pose.position.x
    t_y = odom.pose.position.y
    t_z = odom.pose.position.z
     
    # 4x4 rotation matrix
    mat_drone_odom = np.array([[r00, r01, r02, t_x],
                                [r10, r11, r12, t_y],
                                [r20, r21, r22, t_z],
                                [0,   0,   0,   1]])

    mat_vicon_odom = np.dot(np.linalg.inv(mat_drone_odom),mat_drone_vicon)
    #mat_vicon_odom = np.dot(mat_drone_vicon, np.linalg.inv(mat_drone_odom))
    return mat_vicon_odom
    
'''
  Transform pose in vicon frame to local frame
  @param_in transform: vicon to odom transform matrix
  @param_in pose: pose of target in vicon frame
  @returns pose of target in local frame
'''
def transform_vicon_pose(transform, pose):
    
    # create 4x4 pose transform matrix:
    q0 = pose.orientation.w
    q1 = pose.orientation.x
    q2 = pose.orientation.y
    q3 = pose.orientation.z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    t_x = pose.position.x
    t_y = pose.position.y
    t_z = pose.position.z
     
    # 4x4 rotation matrix
    mat_pose_vicon = np.array([[r00, r01, r02, t_x],
                                [r10, r11, r12, t_y],
                                [r20, r21, r22, t_z],
                                [0,   0,   0,   1]])
    
    mat_pose_local = np.dot(transform,mat_pose_vicon)
    #mat_pose_local = np.dot(mat_pose_vicon, transform)
    pose_local = Pose()
    pose_local.position.x = mat_pose_local[0,3]
    pose_local.position.y = mat_pose_local[1,3]
    pose_local.position.z = mat_pose_local[2,3]

    local_quaternion = quaternion_from_matrix(mat_pose_local)
    pose_local.orientation.x = local_quaternion[0]
    pose_local.orientation.y = local_quaternion[1]
    pose_local.orientation.z = local_quaternion[2]
    pose_local.orientation.w = local_quaternion[3]

    return pose_local


def unit_testing():
    dummy_vicon = TransformStamped()
    dummy_local = PoseStamped()

    dummy_vicon.transform.translation.x = 1
    dummy_vicon.transform.translation.y = 1
    dummy_vicon.transform.translation.z = 0

    dummy_vicon.transform.rotation.x = 0
    dummy_vicon.transform.rotation.y = 0
    dummy_vicon.transform.rotation.z = 0
    dummy_vicon.transform.rotation.w = 1

    dummy_local.pose.position.x = 1
    dummy_local.pose.position.y = -2
    dummy_local.pose.position.z = 0

    dummy_local.pose.orientation.x = 0.7071068
    dummy_local.pose.orientation.y = 0
    dummy_local.pose.orientation.z = 0
    dummy_local.pose.orientation.w = 0.7071068

    vicon_odom_mat = get_vicon_to_odom_transform(dummy_vicon, dummy_local)

    # Target in Vicon
    dummy_target_pose = Pose()
    dummy_target_pose.position.x = -1
    dummy_target_pose.position.y = 0
    dummy_target_pose.position.z = 0
    dummy_target_pose.orientation.x = 0
    dummy_target_pose.orientation.y = 0
    dummy_target_pose.orientation.z = 0
    dummy_target_pose.orientation.w = 1

    result = transform_vicon_pose(vicon_odom_mat, dummy_target_pose)
    print(result)

    # Do the reverse as well:
    dummy_vicon = TransformStamped()
    dummy_local = PoseStamped()

    dummy_vicon.transform.translation.x = -1
    dummy_vicon.transform.translation.y = 0
    dummy_vicon.transform.translation.z = 0

    dummy_vicon.transform.rotation.x = 0
    dummy_vicon.transform.rotation.y = 0
    dummy_vicon.transform.rotation.z = 0
    dummy_vicon.transform.rotation.w = 1

    dummy_local.pose.position.x = -1
    dummy_local.pose.position.y = -2
    dummy_local.pose.position.z = -1

    dummy_local.pose.orientation.x = 0.7071068
    dummy_local.pose.orientation.y = 0
    dummy_local.pose.orientation.z = 0
    dummy_local.pose.orientation.w = 0.7071068

    vicon_odom_mat = get_vicon_to_odom_transform(dummy_vicon, dummy_local)
    
    # Target in Vicon
    dummy_target_pose = Pose()
    dummy_target_pose.position.x = 1
    dummy_target_pose.position.y = 1
    dummy_target_pose.position.z = 0
    dummy_target_pose.orientation.x = 0
    dummy_target_pose.orientation.y = 0
    dummy_target_pose.orientation.z = 0
    dummy_target_pose.orientation.w = 1

    result = transform_vicon_pose(vicon_odom_mat, dummy_target_pose)
    print(result)
    return
