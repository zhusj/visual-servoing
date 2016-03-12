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

import std_srvs.srv

def get_tf_listener():
    import roslib
    roslib.load_manifest('tf')
    import tf
    return tf.TransformListener()

def reset_cameras():
    reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
    rospy.wait_for_service('cameras/reset', timeout=10)
    reset_srv()

def open_right_arm_cam_small():

# Opens the right hand camera on the Baxter at half resolution and publishes the images and camera_info to ROS. 
# The resolution can be tuned as you'd like, although at full resolution the apriltags_ros package approaches 1Hz. 
# If you are using your own image topics, please update the apriltags_ros entry in the launch files.
    try:
        head_camera = baxter_interface.CameraController("head_camera")
        print ("Attempting to turn off the head camera...")
        head_camera.close()
    except Exception:
        pass

	cam = baxter_interface.camera.CameraController("left_hand_camera")

	# close camera
	cam.close()

	head_cam = baxter_interface.camera.CameraController("head_camera")
	head_cam.close()

	x_res       = 960                       # Camera resolution
	y_res       = 600

	# set camera parameters
	cam.resolution          = (int(x_res), int(y_res))
	cam.exposure            = -1             # range, 0-100 auto = -1
	cam.gain                = -1             # range, 0-79 auto = -1
	cam.white_balance_blue  = -1             # range 0-4095, auto = -1
	cam.white_balance_green = -1             # range 0-4095, auto = -1
	cam.white_balance_red   = -1             # range 0-4095, auto = -1

	# open camera
	cam.open()


def get_right_arm_pose():
	# Gets the transformation between the right hand camera and the robot base, so that a velocity in the
	#  camera frame can be applied (as Baxter velocity commands are in base frame). Returns a homogeneous 
	#  translation vector and rotation matrix.
	limb_interface = baxter_interface.Limb("right")
	quaternion_pose = self.limb_interface.endpoint_pose()

	return quaternion_pose


def print_arm_pose(self):
	return
	pi = math.pi

	quaternion_pose = self.limb_interface.endpoint_pose()
	position        = quaternion_pose['position']
	quaternion      = quaternion_pose['orientation']
	euler           = tf.transformations.euler_from_quaternion(quaternion)

	print
	print "             %s" % self.limb
	print 'front back = %5.4f ' % position[0]
	print 'left right = %5.4f ' % position[1]
	print 'up down    = %5.4f ' % position[2]
	print 'roll       = %5.4f radians %5.4f degrees' %euler[0], 180.0 * euler[0] / pi
	print 'pitch      = %5.4f radians %5.4f degrees' %euler[1], 180.0 * euler[1] / pi
	print 'yaw        = %5.4f radians %5.4f degrees' %euler[2], 180.0 * euler[2] / pi