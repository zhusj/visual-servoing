# import the necessary packages
import numpy as np
import argparse
import cv2
import matplotlib.pyplot as plt
 
image = cv2.imread('/home/pracsys/shaojun/visual_servoing_ws/test2.png')
image = cv2.resize(image, (960, 600)) 
output = image.copy()
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# cv2.imshow("input", gray)
# cv2.waitKey(0)

# detect circles in the image
circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 2, 80)
 
# ensure at least some circles were found

print 'circles: ', circles
if circles is not None:
	# convert the (x, y) coordinates and radius of the circles to integers
	circles = np.round(circles[0, :]).astype("int")
 
	# loop over the (x, y) coordinates and radius of the circles
	for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
		cv2.circle(output, (x, y), r, (0, 255, 0), 4)
		cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
 
	# show the output image
	cv2.imshow("output", np.hstack([image, output]))
	cv2.waitKey(0)
	# plt.figure(1)
	# plt.show(plt.imshow("output", np.hstack([image, output])))
	# cv2.waitKey(0)