import cv2
import cv
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('/home/pracsys/shaojun/visual_servoing_ws/expo.png')
print 'shape: ', img.shape
# img1 = cv.fromarray(img)
# plt.figure(1);
# plt.show(plt.imshow(img))
gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# small = cv2.resize(gray, (0,0), fx=0.5, fy=0.5) 
print 'shape: ', gray.shape
# plt.figure(1);
# plt.show(plt.imshow(small))
detector = cv2.SIFT()#FeatureDetector_create("SIFT")
kps = detector.detect(gray)
# print kps

# kps = np.float32([kp.pt for kp in kps])
 
# return a tuple of keypoints and features
# return (kps, features)
extractor = cv2.DescriptorExtractor_create("SIFT")
(kps, features) = extractor.compute(gray, kps)
img=cv2.drawKeypoints(gray,kps)

cv2.imwrite('sift_keypoints.jpg',img)

# surf = cv2.SIFT()
# mask = np.uint8(np.ones(gray.shape))
# surf_points = surf.detect(gray, mask)
