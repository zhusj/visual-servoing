#!/usr/bin/env python
"""
Performs eye in hand (eih) image based visual servoing (ibvs). 
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)
"""
import roslib
import numpy as np
import numpy.matlib
import sys
import rospy
import cv, cv2
from tf.transformations import *
import tf
# Don't include these classes if you aren't using AprilTags or Baxter
# from apriltag_client import AprilTagClient
# from apriltags_ros.msg import AprilTagDetectionArray
from baxter_wrapper import BaxterVS


from visual_servoing import VisualServoing
from utility import *

from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import cv_bridge
import time, os
import baxter

class IbvsEih(object):
    """
    Performs eye in hand (eih) image based visual servoing (ibvs). 
    """
    def __init__(self):
        # Baxter specific code. You don't need this if you're using another robot.
        # Sets the arm that is used for the servoing
        limb='left'
        self._baxter = BaxterVS(limb)

        self.gripper = baxter_interface.Gripper(limb)
        self.gripper.calibrate()

        self.golf_ball_x = 0.85                        # x     = front back
        self.golf_ball_y = 0.10                        # y     = left right
        self.golf_ball_z = 0.0                        # z     = up down
        self.roll        = -1.0 * math.pi              # roll  = horizontal
        self.pitch       = 0.0 * math.pi               # pitch = vertical
        self.yaw         = 0.0 * math.pi               # yaw   = rotation

        self.pose = (self.golf_ball_x,
                        self.golf_ball_y,
                        self.golf_ball_z,
                        self.roll,
                        self.pitch,
                        self.yaw)
        print 'ready to move'
        self.baxter_ik_move(limb, self.pose)
        # self.golf_ball_x = self.golf_ball_x + 0.010
        # # self.golf_ball_y = self.golf_ball_y - 0.10
        # # self.golf_ball_z = self.golf_ball_z + 0.10  
        # self.pitch = self.pitch - 0.2 * math.pi 
        # self.pose = (self.golf_ball_x,
        #                 self.golf_ball_y,
        #                 self.golf_ball_z,
        #                 self.roll,
        #                 self.pitch,
        #                 self.yaw)
        # print 'ready to move'
        # self.baxter_ik_move(limb, self.pose)          
        # AprilTag specific code. You don't need this if you're using another tracking system.
        # Initializes the marker that the arm should track
        target_marker = 0
        # self._apriltag_client = AprilTagClient(target_marker)

        self._visual_servo = VisualServoing(ibvs=True)

        # camera parameters (NB. other parameters in open_camera)
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = -0.01                   # camera gripper offset
        self.cam_y_offset = -0.035
        self.width        = 960                       # Camera resolution
        self.height       = 600

        # callback image
        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)

        # create image publisher to head monitor
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.subscribe_to_camera(limb)
    
    def new_image_arrived(self):
        """
        Boolean to test if a new image has arrived.
        """
        print "waiting for new image"
        if self._apriltag_client.corners is not None:
            self._apriltag_client.corners = None
            return True
        return False

    # def _get_detected_corners(self):
    #     """
    #     Returns the most recently detected corners in the image.
    #     """
    #     # This method currently uses AprilTag detections, so replace with
    #     # your own method if otherwise.
    #     # print "looking for corners"
    #     return self._apriltag_client.corners
    
    def _command_velocity(self,cam_vel,error_norm):
        """
        Move the camera at the specified v and omega in vel (6x1 vector).
        """
        # This method currently commands the Baxter robot from Rethink Robotics.
        #Replace with your own method if using another manipulator.
        
        hand_vel_in_base = self._baxter.cam_to_body(cam_vel)
        # print 'baxter_vel: ', baxter_vel
        self._baxter.set_hand_vel(hand_vel_in_base, error_norm)
    
    def set_target(self,final_camera_depth,desired_corners):
        """
        Sets the final camera depth (Z) and desired position for the tracked features
        at the goal position.
        """


        Z = final_camera_depth
        ideal_cam_pose = np.array([0,0,Z])  
        # print 'ideal_cam_pose: ', ideal_cam_pose
        self._visual_servo.set_target(ideal_cam_pose,None,desired_corners)
    
    def move_to_position(self,final_camera_depth,desiredImg, desiredkps, desiredkps_npa, desiredFeatures,dist_tol):
        """
        Runs one instance of the visual servoing control law. Call when a new
        image has arrived.
        """
        
        # self.set_target(final_camera_depth,desired_corners)
        # r = rospy.Rate(60)
        error_norm = 1000
        hand_pose = baxter.get_arm_pose('left')
        count = 1
        while error_norm>dist_tol and not rospy.is_shutdown() and not hand_pose.position.z < -0.371:
            # if not self.new_image_arrived():
            #     # print " no new_image_arrived"
            #     continue
            
            # Continue if no corners detected
            # marker_corners = self._get_detected_corners()
            # img_array = cv2array(self.cv_image)
            # print self.cv_image.shape
            # cv.WaitKey(1000)
            currentI = self.cv_image
            (detectedkps, detectedkps_npa, detectedFeatures) = self.detectAndDescribe(currentI)

            img=cv2.drawKeypoints(currentI,detectedkps)
            
            # plt.figure(1);
            # plt.show(plt.imshow(img))

            # plt.figure(1);
            # plt.show(plt.imshow(img))

            ratio=0.75
            reprojThresh=4.0
            # print 'desiredkps_npa: ', desiredkps_npa
            # print 'desiredFeatures: ', desiredFeatures
            (matches, H, status) = self.matchKeypoints(desiredkps_npa, detectedkps_npa, desiredFeatures, detectedFeatures, ratio, reprojThresh)
            selected_matches_id = np.where(status != 0)[0]
            # if matches is None:
            #     # print "no marker corners"
            #     continue
            if len(selected_matches_id) < 4:
                print "no enough matching"
                continue

            # selected_matches_id = selected_matches_id[0:3]

            print 'mathces:', len(selected_matches_id)
            # print 'desiredkps_npa: ', desiredkps_npa
            # print 'status: ', status

            # cv2.waitKey(0)

            # print desiredkps_npa.shape
            # print detectedkps_npa.shape
            # print detectedkps_npa
            # print 'mathces:', matches
            # print 'mathces[1]:', matches[1][0]

            desired_corners = np.zeros(len(selected_matches_id)*2)
            detected_corners = np.zeros(len(selected_matches_id)*2)
            # selected_matches = [(0,0),(0,0),(0,0)]

            for i in range(0,len(selected_matches_id)):
                desired_m = desiredkps_npa[matches[selected_matches_id[i]][1]]
                desired_corners[2*i] = desired_m[0]
                desired_corners[2*i+1] = desired_m[1]
                detected_m = detectedkps_npa[matches[selected_matches_id[i]][0]]
                detected_corners[2*i] = detected_m[0]
                detected_corners[2*i+1] = detected_m[1]
                # selected_matches[i] = matches[selected_matches_id[i]] 

            # print 'desired_corners: ', desired_corners
            # print 'detected_corners: ', detected_corners
            # vis = self.drawMatches(desiredImg, currentI, desiredkps_npa, detectedkps_npa, matches, status)
            # print 'selected_matches: ', selected_matches

            self.set_target(final_camera_depth,desired_corners)



            # Don't move if the target hasn't been set
            if not self._visual_servo._target_set:
                print "the target hasn't been set"
                continue
            
            # Get control law velocity and transform to body frame, then send to robot
            cam_vel, error = self._visual_servo.get_next_vel(corners = detected_corners)
            error_norm = np.linalg.norm(error)/(len(detected_corners)/2)
            self._command_velocity(cam_vel, error_norm)
            # print 'servo_vel: ', servo_vel
            # print 'before sleep'
            # print 'error: ', np.linalg.norm(error)
            # print rospy.is_shutdown()
            # r.sleep()
            # cv.WaitKey(1000)
            vis = self.drawMatches(desiredImg, currentI, desiredkps_npa, detectedkps_npa, matches, status)
            # vis = -self.drawMatches(desiredImg, currentI, desiredkps_npa, detectedkps_npa, selected_matches, np.ones(3))
            cv2.imshow("Keypoint Matches", vis)
            filename = '/home/pracsys/shaojun/visual_servoing_ws/'+str(count) +'.jpg'
            cv2.imwrite(filename, vis)
            # cv2.waitKey(0)

            msg = cv_bridge.CvBridge().cv2_to_imgmsg(vis, encoding="bgr8")
            self.pub.publish(msg)

            rospy.sleep(0.8)
            print 'count: ', count
            count = count + 1




        hand_pose = baxter.get_arm_pose('left')
        print 'hand_pose:', hand_pose

        self.golf_ball_x = hand_pose.position.x + 0.05                     # x     = front back
        self.golf_ball_y = hand_pose.position.y                       # y     = left right
        self.golf_ball_z = hand_pose.position.z                        # z     = up down
        self.orientation = hand_pose.orientation

        self.pose = (self.golf_ball_x,
                        self.golf_ball_y,
                        self.golf_ball_z,
                        self.orientation.x,
                        self.orientation.y,
                        self.orientation.z,
                        self.orientation.w)

        print

        print 'ready to move'
        self.baxter_ik_move('left', self.pose)

        time.sleep(1)

        self.gripper.close()
        time.sleep(1)

        hand_pose = baxter.get_arm_pose('left')
        self.golf_ball_x = hand_pose.position.x                       # x     = front back
        self.golf_ball_y = hand_pose.position.y                       # y     = left right
        self.golf_ball_z = hand_pose.position.z+0.03                       # z     = up down
        self.orientation = hand_pose.orientation

        self.pose = (self.golf_ball_x,
                        self.golf_ball_y,
                        self.golf_ball_z,
                        self.orientation.x,
                        self.orientation.y,
                        self.orientation.z,
                        self.orientation.w)

        print 'ready to move'
        self.baxter_ik_move('left', self.pose)

        hand_pose = baxter.get_arm_pose('left')
        self.golf_ball_x = hand_pose.position.x-0.15                       # x     = front back
        self.golf_ball_y = hand_pose.position.y                       # y     = left right
        self.golf_ball_z = hand_pose.position.z                       # z     = up down
        self.orientation = hand_pose.orientation

        self.pose = (self.golf_ball_x,
                        self.golf_ball_y,
                        self.golf_ball_z,
                        self.orientation.x,
                        self.orientation.y,
                        self.orientation.z,
                        self.orientation.w)

        print

        print 'ready to move'
        self.baxter_ik_move('left', self.pose)




    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            # if self.limb == limb:
            limb_interface = baxter_interface.Limb(limb)
            limb_interface.move_to_joint_positions(limb_joints)
            print 'moving'
            # else:
                # self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            # self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        # if self.limb == limb:               # if working arm
        quaternion_pose = limb_interface.endpoint_pose()
        position        = quaternion_pose['position']

        # if working arm remember actual (x,y) position achieved
        self.pose = [position[0], position[1],                                \
                     self.pose[2], self.pose[3], self.pose[4], self.pose[5]]

    def detectAndDescribe(self, image):
        # convert the image to grayscale
        image = cv2.resize(image, (960, 600)) 
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = HSV[:,:,1] #1 or 2

        # plt.figure(1);
        # plt.show(plt.imshow(HSV[:,:,0]))
        # plt.figure(2);
        # plt.show(plt.imshow(HSV[:,:,1]))
        # plt.figure(3);
        # plt.show(plt.imshow(HSV[:,:,2]))
        # cv2.waitKey(0)

        # detect keypoints in the image
        detector = cv2.FeatureDetector_create("SIFT")
        kps = detector.detect(gray)

        # extract features from the image
        extractor = cv2.DescriptorExtractor_create("SIFT")
        (kps, features) = extractor.compute(gray, kps)
 
        # convert the keypoints from KeyPoint objects to NumPy
        # arrays
        kps_npa = np.float32([kp.pt for kp in kps])
 
        # return a tuple of keypoints and features
        return (kps, kps_npa, features)

    def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
        ratio, reprojThresh):
        # compute the raw matches and initialize the list of actual
        # matches
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
        matches = []
 
        # loop over the raw matches
        for m in rawMatches:
            # ensure the distance is within a certain ratio of each
            # other (i.e. Lowe's ratio test)
            if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                matches.append((m[0].trainIdx, m[0].queryIdx))
                    # computing a homography requires at least 4 matches
        if len(matches) > 4:
            # construct the two sets of points
            ptsA = np.float32([kpsA[i] for (_, i) in matches])
            ptsB = np.float32([kpsB[i] for (i, _) in matches])
 
            # compute the homography between the two sets of points
            (H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
                reprojThresh)

            # print 'status: ', status
 
            # return the matches along with the homograpy matrix
            # and status of each matched point
            return (matches, H, status)
 
            # otherwise, no homograpy could be computed
            # return None

    def drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
        # initialize the output visualization image
        (hA, wA) = imageA.shape[:2]
        (hB, wB) = imageB.shape[:2]
        vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
        vis[0:hA, 0:wA] = imageA
        vis[0:hB, wA:] = imageB
 
        # loop over the matches
        for ((trainIdx, queryIdx), s) in zip(matches, status):
            # only process the match if the keypoint was successfully
            # matched
            if s == 1:
                # draw the match
                ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
                ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
                cv2.line(vis, ptA, ptB, (0, 255, 0), 1)
 
        # return the visualization
        return vis


    def subscribe_to_camera(self, camera):
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        print 'subscribe_to_camera: ', camera
        camera_sub = rospy.Subscriber(camera_str, Image, callback)

    # camera call back function
    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError, e:
            print e

        # 3ms wait
        cv.WaitKey(3)

        # left camera call back function
    def left_camera_callback(self, data):
        
        self.camera_callback(data, "Left Hand Camera")

def cv2array(im):
    depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                   cv.IPL_DEPTH_8S: 'int8',
                   cv.IPL_DEPTH_16U: 'uint16',
                   cv.IPL_DEPTH_16S: 'int16',
                   cv.IPL_DEPTH_32S: 'int32',
                   cv.IPL_DEPTH_32F: 'float32',
                   cv.IPL_DEPTH_64F: 'float64'}

    arrdtype=im.depth
    a = numpy.fromstring(im.tostring(),
                         dtype = depth2dtype[im.depth],
                         count = im.width * im.height * im.nChannels)
    a.shape = (im.height, im.width, im.nChannels)

    return a

def main(args):

    rospy.init_node('ibvs_eih')
    ibvseih = IbvsEih()

    # img_array = cv2array(ibvseih.cv_image)
    # print img_array.shape
    # # plt.figure(1);
    # # plt.show(plt.imshow(img_array))
    # msg = cv_bridge.CvBridge().cv2_to_imgmsg(img_array, encoding="bgr8")
    # ibvseih.pub.publish(msg)
    # print 'published'

    # Set desired camera depth and desired feature coordinates as well as distance from goal before stopping
    final_camera_depth = 0.11
    # desired_corners = np.array([10,10,-10,10,10,-10,-10,-10])
    img = cv2.imread('/home/pracsys/shaojun/visual_servoing_ws/expo_shelf.png')
    img = cv2.resize(img, (960, 600)) 
    (desiredkps, desiredkps_npa, desiredFeatures) = ibvseih.detectAndDescribe(img)

    imgs=cv2.drawKeypoints(img,desiredkps)
    
    # plt.figure(1);
    # plt.show(plt.imshow(imgs))
    # cv2.waitKey(0)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    ibvseih.pub.publish(msg)

    dist_tol = 4
    # print desiredkps[0:4]
    # print desiredkps_npa[0:4]
    # print desiredFeatures[0:4]
    ibvseih.move_to_position(final_camera_depth, img, desiredkps, desiredkps_npa, desiredFeatures,dist_tol)



if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
