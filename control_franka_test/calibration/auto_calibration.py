#!/usr/bin/env python2

import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import pyrealsense2 as rs    
import control_test as ct
import rospy, tf, sys, time
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import tf2_ros


# camera matrix of realsense415 with half resolution
camera_info = CameraInfo()
camera_info.K = [931.6937866210938, 0.0, 624.7894897460938, 0.0, 931.462890625, 360.5186767578125, 0.0, 0.0, 1.0]
camera_info.header.frame_id = 'camera_color_optical_frame'
camera_info.height = 720
camera_info.width = 1280

workspace_limits = np.asarray([[0.3-0.08, 0.3+0.08], [-0.524-0.1,-0.524+0.1], [0.5-0.02, 0.5+0.02]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
calib_grid_step = 0.04
checkerboard_size = (3,3)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

calib_grid_step = 0.04
chessboard_size = (3,3)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def detect_one_points():        
        Temp_pts = np.zeros(3)
        scan_number = 10
        i = 0
        while i < scan_number:
                align_to = rs.stream.color
                align = rs.align(align_to)
                for m in range(5):
                        frames = pipeline.wait_for_frames()

                frames = pipeline.wait_for_frames()
                #print(i)
                aligned_frames = align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
                chessboard_found, corners = cv2.findChessboardCorners(gray,chessboard_size,None)
                
                try:
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
                                Temp_pts[0] = Temp_pts[0]+observed_pt[0]       
                                Temp_pts[1] = Temp_pts[1]+observed_pt[1]
                                Temp_pts[2] = Temp_pts[2]+observed_pt[2]
                                i = i + 1
                except Exception:
                        pass 
        observed_pt = Temp_pts/scan_number 
        
        print(observed_pt)  
           
        # Draw and display the corners
        cv2.drawChessboardCorners(color_image, (1,1), corners[4], chessboard_found)
        cv2.imshow('Corner_center',color_image)
        cv2.waitKey(1)                       
        return observed_pt

def image_callback(color_image, depth_image):
        gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
        if checkerboard_found:
                cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), refine_criteria)

                # Get observed checkerboard center 3D point in camera space
                checkerboard_pix = np.round(corners[4,0,:]).astype(int)
                checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1]-20:checkerboard_pix[1]+20,checkerboard_pix[0]-20:checkerboard_pix[0]+20])) * depth_scale
                print("Found checkerboard, Z = ", checkerboard_z)
                checkerboard_x = np.multiply(checkerboard_pix[0]-624.79,checkerboard_z/931.69)
                checkerboard_y = np.multiply(checkerboard_pix[1]-360.52,checkerboard_z/931.46)
                if checkerboard_z > 0:
                    # Save calibration point and observed checkerboard center
                    observed_pt = np.array([checkerboard_x,checkerboard_y,checkerboard_z])
                    print('obesrved_pt:',observed_pt)
                # Draw and display the corners
                vis = cv2.drawChessboardCorners(color_image, (1,1), corners[4,:,:], checkerboard_found)
                #cv2.imwrite('images/color_image_checkerboard_%02d.png' % iter, vis)
        return observed_pt        

# Setup:
points = rs.points()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align_to = rs.stream.color
align = rs.align(align_to)

frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)

aligned_depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()

depth_image = np.asanyarray(aligned_depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())
for m in range(5):
        frames = pipeline.wait_for_frames()
print(depth_image.shape)
print(color_image.shape)
# init panda
panda = ct.my_panda()

measured_pts = []
observed_pts = []
x_start = 0.35750
y_start = -0.02582
z_start = 0.19375
x_end = 0.66545
y_end = 0.36548
z_end = 0.4
x_range = x_end - x_start
y_range = y_end - y_start
z_range = z_end - z_start
iter_i = 4
iter_j = 4
iter_k = 4
for i in range(iter_i):
        x = x_start + i * (x_range/(iter_i-1))
        for j in range(iter_j):
                y = y_start + j * (y_range/(iter_j-1))
                for k in range(iter_k):
                        z = z_start + k * (z_range/(iter_k-1))
                        measured_pt = [x, y, z]
                        goal = panda.set_goal(measured_pt)
                        panda.move_arm(goal)
                        frames = pipeline.wait_for_frames()
                        aligned_frames = align.process(frames)

                        aligned_depth_frame = aligned_frames.get_depth_frame()
                        color_frame = aligned_frames.get_color_frame()

                        depth_image = np.asanyarray(aligned_depth_frame.get_data())
                        color_image = np.asanyarray(color_frame.get_data())
                        observed_pt = image_callback(color_image, depth_image)
                        measured_pts = measured_pts + [measured_pt]
                        observed_pts = observed_pts + [observed_pt]
                        print('move to', i, j, k)
observed_pts = np.array(observed_pts)
measured_pts = np.array(measured_pts)
np.savez("calibration_data.npz",observed_pts,measured_pts)

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
print('finished')
print(x)
