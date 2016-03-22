#!/usr/bin/env python
"""
Wrapper class to interface baxter python functions (possibly GRASP specific) with visual servoing needs.
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)
"""
import baxter
import baxter_interface
from baxter_pykdl import baxter_kinematics

import numpy as np
import roslib
import rospy
import sys
from tf.transformations import quaternion_matrix
import tf
from utility import *
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


class BaxterVS(object):
    """
    Wrapper class to interface baxter python functions (possibly GRASP specific) with visual servoing needs.
    """
    def __init__(self,limb):
        # Personal function to open left hand camera at half resolution. Replace with your own as needed.
        # (The apriltags_ros package is too slow with full resolution).
        baxter.reset_cameras()
        baxter.open_left_arm_cam_small()

        transform=baxter.get_tf_listener()
        transform.waitForTransform('/' + limb + '_hand','/' + limb + '_hand_camera',rospy.Time(0),rospy.Duration(5.0))
        (self._cam2hand_t,self._cam2hand_R)=transform.lookupTransform('/' + limb + '_hand','/' + limb + '_hand_camera',rospy.Time(0))
        self._cam2hand_t= np.concatenate((np.transpose(np.matrix(self._cam2hand_t)),np.matrix([[0]])),axis=0)
        self._cam2hand_R=quaternion_matrix(self._cam2hand_R)

        self._arm=baxter_interface.limb.Limb(limb)
        self._kin = baxter_kinematics(limb)
        self._limb = limb

    def cam_to_body(self, vector):
        """
        Returns the transformation between the left camera to the base, 
        for applying direct twist vectors using the baxter API.
        """
        cam2hand = generate_frame_transform(self._cam2hand_t[0:3,:],self._cam2hand_R[0:3,0:3],False)
        # Possibly GRASP specific function?
        hand_pose = baxter.get_arm_pose(self._limb)
        # print 'hand_pose:', hand_pose
        print 'hand_pose.position:', hand_pose.position
        (t,R) = get_t_R(hand_pose)
        # print 't:', t
        hand2base = generate_frame_transform(t[0:3,:],R[0:3,0:3],True)
        return np.dot(hand2base,np.dot(cam2hand,vector))
        # return np.dot(cam2hand,vector)
        
    def set_hand_vel(self,vel,error_norm):
        """
        Given a 6x1 twist vector, sets the corresponding joint velocities using the PyKDL package.
        """
         # Calculate joint velocities to achieve desired velocity
        
        joint_vels=np.dot(self._kin.jacobian_pseudo_inverse(),vel)

        # threshold = min(0.3, error_norm/30)
        # while any(np.abs(i) >= threshold for i in joint_vels):
        #     joint_vels = joint_vels * 0.8

        joints=dict(zip(self._arm.joint_names(),(joint_vels)))
        
        # print 'joints: ', joints

        self._arm.set_joint_velocities(joints)
