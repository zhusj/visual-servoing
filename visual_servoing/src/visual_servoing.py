#!/usr/bin/env python
"""
General visual servoing class to perform either image based visual servoing (ibvs) or 
pose based visual servoing (pbvs). Currently only eye in hand (eih) methods are supported,
although eye to hand (eth) methods are easily applied by applying the transformation from 
the camera (eye) to the hand to the velocity twist vector.
"""
import roslib
import numpy as np
import numpy.matlib
import cv2
import sys
import rospy
from tf.transformations import *

from utility import *

from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Point32,
    Quaternion,
)

import struct

import baxter

class VisualServoing(object):
    """
    General visual servoing class to perform either image based visual servoing (ibvs) or 
    pose based visual servoing (pbvs). Currently only eye in hand (eih) methods are supported,
    although eye to hand (eth) methods are easily applied by applying the transformation from 
    the camera (eye) to the hand to the velocity twist vector.
    """
    def __init__(self, ibvs):
        self._translation_only=False
        # Set to true to set all output velocities to test_vel (arm moves according to test_vel).
        self._test_servoing=False
        self._test_vel = np.array([[0.1],[0],[0],[0],[0],[0]])
        # self._L=np.matlib.zeros((2*4,6))
        # self._ideal_feature=np.matlib.zeros((4*2,1))
        # Performs ibvs if true, pbvs else.
        self._ibvs = ibvs

        # Gain on controller, essentially sets arm speed, although too high of a value will cause the
        # function to diverge.
        self._lambda = 0.6

        self._target_set=False
        
    def set_target(self,ideal_cam_pose=None, ideal_cam_rot=None,ideal_corners=None):
        """
        Sets the target position for the visual servoing law. The pose inputs are in homogeneous coordinates.
        While the corner positions aren't necessary for pbvs, they are still used to draw the desired position
        in the image.
        """
        self._ideal_cam_pose=ideal_cam_pose
        self._ideal_cam_rot = ideal_cam_rot
        if ideal_corners is not None:
            self._ideal_corners = ideal_corners
        if self._ibvs:
            self._eih_initialize_target_feature()
        if not self._ibvs:
            self._ideal_feature = self._calc_feature(ideal_cam_pose,ideal_cam_rot)
        self._target_set=True
 
    def _shutdown_hook(self):
        pass

    def _eih_initialize_target_feature(self):
        """
        In the event of ibvs eih servoing, initialize the interaction matrix (L) based on
        the desired position. The same L matrix will be used as an approximation to the true
        L throughout the servoing process (so that we don't have to reestimate the depth Z
        at each step). While estimating the depth is possible with the tags, it is useful to
        experiment with a constant interaction matrix regardless.
        """
        self._ideal_feature = np.matlib.zeros((len(self._ideal_corners),1))
        self._L=np.matlib.zeros((len(self._ideal_corners),6))
        gamma = 410
        for i in range(0,len(self._ideal_corners)/2):
            x=self._ideal_corners[i*2]
            y=self._ideal_corners[i*2+1]
            self._ideal_feature[i*2,0]=self._ideal_corners[i*2]
            self._ideal_feature[i*2+1,0]=self._ideal_corners[i*2+1]
            p = self._ideal_cam_pose
            # print 'Z: ', p[2]
            Z=p[2]
            self._L[i*2:i*2+2,:]=np.matrix([[-gamma/Z,0,x/Z,x*y/gamma,-gamma-x*x/gamma,y],[0,-gamma/Z,y/Z,gamma+y*y/gamma,-x*y/gamma,-x]])

    def _generate_L(self,t,R):
        """
        Used for pbvs only. Generate the interaction matrix L at each step.
        """
        (theta,u,p)=rotation_from_matrix(R)

        L_top=np.concatenate((-np.identity(3),generate_skew_mat(t)),axis=1)
        
        L_bottom=np.concatenate((np.zeros((3,3)),(np.identity(3)-theta/2*generate_skew_mat(u)+(1-(np.sinc(theta)/(np.sinc(theta/2)*np.sinc(theta/2))))*np.dot(generate_skew_mat(u),generate_skew_mat(u)))),axis=1)
        
        L=np.concatenate((L_top,L_bottom),axis=0)
        
        return L
    
    def _calc_feature(self,t,R):
        """
        Used for pbvs only. Computes the feature vector given an input pose.
        """
        R_rotated=np.dot(R.T,(self._ideal_cam_rot))
        (theta,u,p)=rotation_from_matrix(R_rotated)
        if self._translation_only:
            feature=np.concatenate((t[0:3,0],np.zeros((3,1))),axis=0)
        else:
            feature=np.concatenate((t[0:3,0],theta*u[:,None]),axis=0)
        return feature
    
    def get_next_vel(self,t=None,R=None,corners=None):
        """
        Computes the servo law mandated velocity given a current pose or set of image coordinates.
        At least one of either t and R or corners must be input.
        """
        if (t is None or R is None) and corners is None:
            return
        if self._ibvs:
            target_feature = corners.flatten()
            target_feature = target_feature[:,None]
            L = self._L
        else:
            L = self._generate_L(t,R)
            target_feature = self._calc_feature(t,R)
        error = target_feature - self._ideal_feature
        error_norm = np.linalg.norm(error)/(len(corners)/2)
        # print 'target_feature: ', target_feature[0:6]
        # print 'self._ideal_feature: ', self._ideal_feature[0:6]
        # print 'error: ', error
        print 'error norm: ', error_norm
        # print self._L
        
        # print 'L inv: ', np.linalg.pinv(L)
        # print 'error:', np.linalg.norm(error)/len(error)

        self._Lc=np.matlib.zeros((len(corners),6))

        hand_pose = baxter.get_arm_pose('left')
        # print 'hand_pose:', hand_pose
        Z = 0.3+hand_pose.position.z
        print 'Z: ', Z 
        gamma =410
        for i in range(0,len(corners)/2):
            x=corners[i*2]
            y=corners[i*2+1]
            p = self._ideal_cam_pose
            # print 'Z: ', p[2]
            # Z=p[2]
            self._Lc[i*2:i*2+2,:]=np.matrix([[-gamma/Z,0,x/Z,x*y/gamma,-gamma-x*x/gamma,y],[0,-gamma/Z,y/Z,gamma+y*y/gamma,-x*y/gamma,-x]])
        
        L = (L+self._Lc)/2
        # L = self._Lc
        # print np.dot(np.linalg.pinv(L),error)
        # print 'L: ',L
        

        vel = -self._lambda*np.dot(np.linalg.pinv(L),error)

        # if error_norm < 11:
           

        # if hand_pose.position.z < -0.135 and error_norm > 9:
        #     # vel[0] = 3* vel[0]
        #     # vel[1] = 3* vel[1]
        #     vel[2] = 0
            # vel[5] = 0
        # ratio = max(np.abs(vel))#error_norm/100
        # thresh = 0.05
        # while np.abs(vel[0]) > thresh or np.abs(vel[1]) > thresh:
        #     print 'out of limit'
        #     print 'cam_vel: ', vel
        #     if math.isinf(np.abs(vel[0])):
        #         vel = np.zeros(6)
        #         continue
        #     vel = vel/ratio*0.05

        # if np.linalg.norm(error) < 300:
        #     vel = -0.01*np.dot(np.linalg.pinv(L),error)
        # else:
        #     vel = -self._lambda*np.dot(np.linalg.pinv(L),error)
        # vel[0] = vel[0]
        # vel[1] = - vel[1]
        
        # vel = 0*np.dot(np.linalg.pinv(L),error)
        # vel[0] = 0.1
        # vel[1] = -0.1
        print 'cam_vel: ', vel
        return vel, error
