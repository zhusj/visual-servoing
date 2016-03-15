# #!/usr/bin/env python
# import baxter_interface
# import rospy
# from sensor_msgs.msg import Image


# rospy.init_node("my_cam")
# display_pub= rospy.Publisher('/robot/xdisplay',Image)
# def republish(msg):
#         """
#             Sends the camera image to baxter's display
#         """             
#         display_pub.publish(msg)
# head_camera = baxter_interface.CameraController("head_camera")
# # head_camera.close()
# left_camera = baxter_interface.CameraController("left_hand_camera")

# left_camera.resolution =(960, 600)
# left_camera.open()
# camera_name = "left_camera"
# print camera_name
# sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
# rospy.spin()

#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image


rospy.init_node("my_cam")
display_pub= rospy.Publisher('/robot/xdisplay',Image)
def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
        display_pub.publish(msg)
left_camera = baxter_interface.CameraController("left_hand_camera")
left_camera.close()
head_camera = baxter_interface.CameraController("head_camera")

head_camera.resolution =(960, 600)
head_camera.open()
camera_name = "head_camera"
sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
rospy.spin()