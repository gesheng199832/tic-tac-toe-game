#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformerROS, transformations as tfs
import tf
from geometry_msgs.msg import Transform
import numpy as np 
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
from scipy.linalg import lstsq
from numpy import *

print("Environment Ready")
'''
calib_grid_step = 0.04
chessboard_size = (3,3)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def brighter(image, percetage=1.5):
    image_copy = image.copy()
    w = image.shape[1]
    h = image.shape[0]
    #get brighter
    for xi in range(0,w):
        for xj in range(0,h):
            image_copy[xj,xi,0] = np.clip(int(image[xj,xi,0]*percetage),a_max=255,a_min=0)
            image_copy[xj,xi,1] = np.clip(int(image[xj,xi,1]*percetage),a_max=255,a_min=0)
            image_copy[xj,xi,2] = np.clip(int(image[xj,xi,2]*percetage),a_max=255,a_min=0)
    return image_copy

# Setup:
points = rs.points()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
#print("Depth Scale is: " , depth_scale)
align_to = rs.stream.color
align = rs.align(align_to)

frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)

aligned_depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()

depth_image = np.asanyarray(aligned_depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())

#print(depth_image.shape)
#print(depth_image)
#cv2.imwrite('images/color_image_%02d.png' % iter, color_image)
#images = np.hstack((color_image, depth_image))
#colorizer = rs.colorizer()
#colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
#plt.imshow(color_image)

#plt.imshow(depth_image)
#plt.show()
#plt.imshow(color_image)
#color_image = brighter(color_image, percetage=1.5)
#cv2.imshow('abc',color_image)
#cv2.waitKey(0)
observed_pts = []
gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
chessboard_found, corners = cv2.findChessboardCorners(gray,chessboard_size,None)
if chessboard_found:
        # refine the corners if chessboard is found
        cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),refine_criteria)
        # Get observed chessboard center 3D point in camera space
        observed_pt = corners[4]
        observed_pt = observed_pt[0]
        n = 0
        depth_value = 0
        step = 1
        for x in range(5):
            for y in range(5):
                depth_pre = depth_image[int(observed_pt[1])-step*(2-x)][int(observed_pt[0])-step*(2-y)]
                if depth_pre != 0:
                    depth_value = depth_value + depth_pre
                    n = n + 1
        depth_value = depth_value/n
        observed_pt = np.append(observed_pt, depth_value)
        observed_pts.append(observed_pt)
        #print('observed_pt',observed_pt)
        # Draw and display the corners
        cv2.drawChessboardCorners(color_image, (1,1), corners[4], chessboard_found)
        cv2.imshow('Corner_center',color_image)
cv2.waitKey(0)
#cv2.destoryAllWindows()
#plt.show()
pipeline.stop()
'''
d = np.load("calibration_data.npz")
observed_pts = d['arr_0']
measured_pts = d['arr_1']

# TODO: define function get_rigid_transform to calculate the transformation between two sets of point A and B (AX = B)
def get_rigid_transform(A, B):
    one = np.ones((len(A), 1))
    A = np.column_stack([A, one])
    B = np.column_stack([B, one])
    x, foo1, foo2, foo3 = np.linalg.lstsq(A, B)
    #print(x)
    x = x.T
    R = x[0:3, 0:3]
    T = x[0:3, 3:4]    
    return x,R,T

x, R, t = get_rigid_transform(observed_pts, measured_pts)
'''
pt = np.zeros((4,1))
observed_pt = np.append(observed_pt, 1)
observed_pt = mat(observed_pt)
x = mat(x)
print(x)
print(R)
print(t)
print(observed_pt)
f = x.dot(observed_pt.T)
'''

print(x)







