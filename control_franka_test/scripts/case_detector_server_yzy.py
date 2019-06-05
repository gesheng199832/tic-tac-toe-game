#!/usr/bin/env python2
#coding:utf-8　
import cv2
import numpy as np
import math
from math import pi
from math import sin
from math import cos
from math import tan
import copy
import pyrealsense2 as rs 
from control_franka_test.srv import *
import rospy
import tf, sys, time
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import tf2_ros

# Constants
left = 400
upper = 200
right = 900
lower = 660

# Color Fliters
lower_blue1 = np.array([70,60,90])
upper_blue1 = np.array([255,110,139]) 

lower_red1 = np.array([50,60,140])
upper_red1 = np.array([95,110,255])

#lower_blue2 = np.array([70,70,90])
#upper_blue2 = np.array([255,110,139]) 

#lower_red2 = np.array([75,70,140])
#upper_red2 = np.array([95,110,255])
lower_red2 = np.array([25,20,160])
upper_red2 = np.array([50,50,255])

lower_white = np.array([200,200,200])
upper_white = np.array([255,255,255])

# Setup Realsense:
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

# Global variblies
lastboardcase = None
#firstrectangle = None
firstrectangle = [(640,320),(100,100),0]
#firstbox = None
firstbox = np.array([[640+50,360+50],[640+50,360-50],[640-50,360-50],[640-50,360+50]])

#Video setup
fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280,720))


def crossORcircle(onechess_image,pt):
        global image
        coc = 0       
                        
        blue_mask = cv2.inRange(onechess_image, lower_blue1, upper_blue1) 
       
        red_mask = cv2.inRange(onechess_image, lower_red1, upper_red1)
        
        #cv2.imshow('one',onechess_image)  
        #cv2.imshow('redmask',red_mask) 
        #cv2.imshow('bluemask',blue_mask)      
        
        height, width, thickness = onechess_image.shape 
        area = height * width * 255
        bluearea = 0
        redarea = 0
        for i in range(height): 
                for j in range(width): 
                        bluearea = bluearea + blue_mask[i,j]
                        redarea = redarea + red_mask[i,j]
        bluescore = float(bluearea) / area  *100
        redscore = float(redarea) / area *100       
        if bluescore > redscore:
                #print('circle')
                cv2.putText(image,'o',pt,cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
                coc = 1
        else:
                #print('cross')
                cv2.putText(image,'x',pt,cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
                coc = 2
                        
        #print('bluescore:',bluescore)
        #print('redscore:',redscore)
        #cv2.waitKey(0)   
        return coc                

def whereischess(board_image,pts,r): #r: length of lattice
        global image
        chessboard = np.array([[0,0,0],[0,0,0],[0,0,0]])
        for row in range(3):
                for col in range(3):
                        height, width, thickness = board_image.shape
                        lattice_mask = np.zeros((height,width), np.uint8)
                        cv2.circle(lattice_mask,pts[row][col],r,(255,255,255),-1)  
                        lattice_mask = cv2.cvtColor(lattice_mask, cv2.COLOR_GRAY2BGR) 
                        onelattice = cv2.bitwise_and(lattice_mask, board_image) #color image                        
                        
                        rowa = pts[row][col][1]
                        colb = pts[row][col][0]                       
                        
                        roi = onelattice[rowa-r-2:rowa+r+2,colb-r-2:colb+r+2]
                        
                        #cv2.namedWindow('roi',0)                       
                        #cv2.imshow('roi',roi)
                        #cv2.imshow('onelattice',onelattice)                             
               
                        blue_mask = cv2.inRange(roi, lower_blue1, upper_blue1)                                 
                        
                        red_mask = cv2.inRange(roi, lower_red1, upper_red1)
                        
                        white_mask = cv2.inRange(roi, lower_white, upper_white)
                        
                        red_mask2 = cv2.inRange(roi, lower_red2, upper_red2)
                                               
                        #cv2.imshow('redmask',red_mask)
                        #cv2.imshow('bluemask',blue_mask)
                        
                        height, width, thickness = roi.shape 
                        area = height * width * 255
                        bluearea = 0
                        redarea = 0
                        whitearea = 0
                        redarea2 = 0
                        for i in range(height): 
                                for j in range(width): 
                                        bluearea = bluearea + blue_mask[i,j]
                                        redarea = redarea + red_mask[i,j]
                                        whitearea = whitearea + white_mask[i,j]
                                        redarea2 = redarea2 + red_mask2[i,j]

                        bluescore = float(bluearea) / area  *100
                        redscore = float(redarea) / area *100       
                        whitescore = float(whitearea) / area *100  
                        redscore2 = float(redarea2) / area *100
                        
                        #print('bluescore:',bluescore)
                        #print('redscore:',redscore)
                        #print('whitescore:',whitescore)
                        #print('redscore2:',redscore2)
                        if whitescore > 10:
                                #print('No chess')
                                pass
                        elif redscore2 > 10:
                                #print('No chess')
                                pass
                        elif (redscore == 0) & (bluescore == 0):                  
                                #print('No chess')
                                pass
                        elif bluescore > redscore:
                                #print('circle')
                                chessboard[row][col] = 1
                                cv2.putText(image,'o',(colb,rowa),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
                        else:
                                #print('cross')
                                chessboard[row][col] = 2
                                cv2.putText(image,'x',(colb,rowa),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
                                                     
                        #cv2.waitKey(0)
                                          
        #print(chessboard)
        return chessboard
        
        
def get_contours(img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,threshold = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)                        
        contours, hier = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                
        global firstrectangle
        global firstbox
        rectangle = None
        box = None
        chess_dict = {}
        number = 0
        for c in contours:    
                # calculate center and radius of minimum enclosing circle
                (x, y), radius = cv2.minEnclosingCircle(c) 
                center = (int(x), int(y))
                radius = int(radius)
                #print(radius)
                if ((radius>= 12) & (radius <=15)):
                        # draw the circle
                        #cv2.circle(img, center, radius+2, (0, 255, 0), 1)
                        chess_dict[str(number)]=[int(radius),int(x),int(y)]
                        number = number + 1    
                else: 
                        temprect = cv2.minAreaRect(c)
                        #print(temprect)                
                        if (temprect[1][0] >= 92) & (temprect[1][0] <=108) & (temprect[1][1] >= 92) & (temprect[1][1] <=108):           
                                rectangle = temprect
                                box = cv2.cv.BoxPoints(rectangle)
                                box =np.int0(box)
                                if firstrectangle is None:
                                        firstbox = box
                                        firstrectangle = rectangle                                 
                                #cv2.drawContours(img, [box], -1, (0, 255, 0), 2) 
        if rectangle is None:
                rectangle = firstrectangle
                box = firstbox
        return chess_dict,rectangle,box



def get_boardpos(img,depth_img):
        chess_dict,rect,box = get_contours(img)
        lattice_l = ((rect[1][0]+rect[1][1])/2*6.4/7)/3
        angle = rect[2]

        #obtain each coordinate of central point of lattice
        center = (int(rect[0][0]+0.5),int(rect[0][1]+0.5))
        #edge points
        pt1 = (int(rect[0][0]+0.5+cos(pi*angle/180)*lattice_l),int(rect[0][1]+0.5+sin(pi*angle/180)*lattice_l))
        pt2 = (int(rect[0][0]+0.5-cos(pi*angle/180)*lattice_l),int(rect[0][1]+0.5-sin(pi*angle/180)*lattice_l))
        pt3 = (int(rect[0][0]+0.5+cos(pi*angle/180+pi/2)*lattice_l),int(rect[0][1]+0.5+sin(pi*angle/180+pi/2)*lattice_l))
        pt4 = (int(rect[0][0]+0.5-cos(pi*angle/180+pi/2)*lattice_l),int(rect[0][1]+0.5-sin(pi*angle/180+pi/2)*lattice_l))
        #corner points
        pt5 = (int(rect[0][0]+0.5+(cos(pi*angle/180)+cos(pi*angle/180+pi/2))*lattice_l),int(rect[0][1]+0.5+(sin(pi*angle/180)+sin(pi*angle/180+pi/2))*lattice_l))
        pt6 = (int(rect[0][0]+0.5-(cos(pi*angle/180)+cos(pi*angle/180+pi/2))*lattice_l),int(rect[0][1]+0.5-(sin(pi*angle/180)+sin(pi*angle/180+pi/2))*lattice_l))
        pt7 = (int(rect[0][0]+0.5+(cos(pi*angle/180+pi/2)-cos(pi*angle/180))*lattice_l),int(rect[0][1]+0.5+(sin(pi*angle/180+pi/2)-sin(pi*angle/180))*lattice_l))
        pt8 = (int(rect[0][0]+0.5-(cos(pi*angle/180+pi/2)-cos(pi*angle/180))*lattice_l),int(rect[0][1]+0.5-(sin(pi*angle/180+pi/2)-sin(pi*angle/180))*lattice_l))

        #board_pts =[[pt6,pt4,pt8],[pt2,center,pt1],[pt7,pt3,pt5]] 
        #print(board_pts) 
        pos = {'0': [pt6[0],pt6[1],depth_img[pt6[1]][pt6[0]]],
                '1': [pt4[0],pt4[1],depth_img[pt4[1]][pt4[0]]],
                '2': [pt8[0],pt8[1],depth_img[pt8[1]][pt8[0]]],
                '3': [pt2[0],pt2[1],depth_img[pt2[1]][pt2[0]]],
                '4': [center[0],center[1],depth_img[center[1]][center[0]]],
                '5': [pt1[0],pt1[1],depth_img[pt1[1]][pt1[0]]],
                '6': [pt7[0],pt7[1],depth_img[pt7[1]][pt7[0]]],
                '7': [pt3[0],pt3[1],depth_img[pt3[1]][pt3[0]]],
                '8': [pt5[0],pt5[1],depth_img[pt5[1]][pt5[0]]]}

        for key in pos:
                temp_z = np.mean(np.mean(depth_img[pos[key][1]-5:pos[key][1]+5,pos[key][0]-5:pos[key][0]+5])) * depth_scale
                temp_x = np.multiply(pos[key][0]-624.79,temp_z/931.69)
                temp_y = np.multiply(pos[key][1]-360.52,temp_z/931.46)
                pos[key] = [temp_x,temp_y,temp_z]
        #print(pos)       
        return pos

def get_chess(img,depth_img,which):
        if which == 'x':
                jvalue = 2
        elif which == 'o':
                jvalue = 1
        else:
                raise Exception,"Invalid input! Please use \'x\' or \'o\'"
        chess_dict,rect,box = get_contours(img)
        chess_point = None
        #decide the chesses outside the board
        for key in chess_dict:
                chess_mask = np.zeros((img.shape[0],img.shape[1]), np.uint8) 
                cv2.circle(chess_mask,(chess_dict[key][1],chess_dict[key][2]),chess_dict[key][0]-1,(255,255,255),-1) 
                                                              
                chess_mask = cv2.cvtColor(chess_mask, cv2.COLOR_GRAY2BGR) 
                
                onechess = cv2.bitwise_and(chess_mask, img) #color image
            
                row = chess_dict[key][2]
                col = chess_dict[key][1]
                r = chess_dict[key][0]    
                roi = onechess[row-r-2:row+r+2, col-r-2:col+r+2]    
                #cv2.imshow('chess'+key,roi)
                chess_pt = (chess_dict[key][1],chess_dict[key][2]) 
                #cv2.putText(image1,str(chess_pt),chess_pt,cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)   
                coc = crossORcircle(roi,chess_pt) 
                if coc == jvalue:
                        depth = depth_img[chess_pt[1]][chess_pt[0]]
                        chess_point = [chess_pt[0],chess_pt[1],depth]
                        #$break                
        #if chess_point is None:
                #raise Exception,"No " + which + " chess"
                #print('No '+ which +' chess')
        if not chess_point is None:
                temp_z = np.mean(np.mean(depth_img[chess_point[1]-5:chess_point[1]+5,chess_point[0]-5:chess_point[0]+5])) * depth_scale
                temp_x = np.multiply(chess_point[0]-624.79,temp_z/931.69)
                temp_y = np.multiply(chess_point[1]-360.52,temp_z/931.46)
                chess_point = [temp_x,temp_y,temp_z]
        
        return chess_point    

def get_board(img):
        global lastboardcase 
        chess_dict,rect,box = get_contours(img)            
        lattice_l = ((rect[1][0]+rect[1][1])/2*6.4/7)/3
        angle = rect[2]

        #obtain each coordinate of central point of lattice
        center = (int(rect[0][0]+0.5),int(rect[0][1]+0.5))
        #edge points
        pt1 = (int(rect[0][0]+0.5+cos(pi*angle/180)*lattice_l),int(rect[0][1]+0.5+sin(pi*angle/180)*lattice_l))
        pt2 = (int(rect[0][0]+0.5-cos(pi*angle/180)*lattice_l),int(rect[0][1]+0.5-sin(pi*angle/180)*lattice_l))
        pt3 = (int(rect[0][0]+0.5+cos(pi*angle/180+pi/2)*lattice_l),int(rect[0][1]+0.5+sin(pi*angle/180+pi/2)*lattice_l))
        pt4 = (int(rect[0][0]+0.5-cos(pi*angle/180+pi/2)*lattice_l),int(rect[0][1]+0.5-sin(pi*angle/180+pi/2)*lattice_l))
        #corner points
        pt5 = (int(rect[0][0]+0.5+(cos(pi*angle/180)+cos(pi*angle/180+pi/2))*lattice_l),int(rect[0][1]+0.5+(sin(pi*angle/180)+sin(pi*angle/180+pi/2))*lattice_l))
        pt6 = (int(rect[0][0]+0.5-(cos(pi*angle/180)+cos(pi*angle/180+pi/2))*lattice_l),int(rect[0][1]+0.5-(sin(pi*angle/180)+sin(pi*angle/180+pi/2))*lattice_l))
        pt7 = (int(rect[0][0]+0.5+(cos(pi*angle/180+pi/2)-cos(pi*angle/180))*lattice_l),int(rect[0][1]+0.5+(sin(pi*angle/180+pi/2)-sin(pi*angle/180))*lattice_l))
        pt8 = (int(rect[0][0]+0.5-(cos(pi*angle/180+pi/2)-cos(pi*angle/180))*lattice_l),int(rect[0][1]+0.5-(sin(pi*angle/180+pi/2)-sin(pi*angle/180))*lattice_l))

        board_pts =[[pt6,pt4,pt8],[pt2,center,pt1],[pt7,pt3,pt5]] 

        mask = np.zeros((img.shape[0],img.shape[1]), np.uint8)
        cv2.drawContours(mask, [box], -1, (255, 255, 255), -1)
        cv2.drawContours(mask, [box], -1, (255, 255, 255), 3)            
        board_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) 
        oneboard = cv2.bitwise_and(board_mask, img) #color image 

        chessboard = whereischess(oneboard, board_pts, 12)                            
        lastboardcase = chessboard
        return chessboard   



# servicv node
def handle_getboardcase(req):
        global image
        global depth_image
        if req.call == 1:
                chess_location = get_chess(image,depth_image,'o')
        elif req.call == 2:
                chess_location = get_chess(image,depth_image,'x') 
        else:
                print('Invalid Input for calling get_chess()!')       
        chessboard = get_board(image)
        boardcase =[chessboard[0][0],chessboard[0][1],chessboard[0][2],chessboard[1][0],chessboard[1][1],chessboard[1][2],chessboard[2][0],chessboard[2][1],chessboard[2][2]]         
        pos = get_boardpos(image,depth_image)
        #cv2.imshow('current_image',image)              
        return [pos['0'],pos['1'],pos['2'],pos['3'],pos['4'],pos['5'],pos['6'],pos['7'],pos['8'],chess_location,boardcase]

# mouse callback function
def get_color(event,x,y,flags,param):
    global image
    if event == cv2.EVENT_LBUTTONDOWN:
        print(image[y][x])
        cv2.circle(image,(x,y),1,(0,255,0),-1)

  
if __name__ == "__main__":
        #Video setup
        fourcc = cv2.cv.CV_FOURCC(*'XVID')
        out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280,720))
        
        # ROS node initialization
        rospy.init_node('chessboard_detector_server', anonymous = True)
        s1 = rospy.Service('boardcase', board_detector, handle_getboardcase)  
       
        cv2.namedWindow('realsense')
        cv2.setMouseCallback('realsense',get_color)
        #skip first 20 frames
        for m in range(20):
                frames = pipeline.wait_for_frames()    
                    
        while(True):
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                image1 = np.asanyarray(color_frame.get_data())
                shape = image1.shape
                out.write(image1)
                #cv2.imshow('realsense',image1)
                       
                # Cut frame
                image1[0:upper,0:shape[1]] = [0,0,0]
                image1[upper:lower,0:left] = [0,0,0]
                image1[upper:lower,right:shape[1]] = [0,0,0]
                image1[lower:shape[0],0:shape[1]] = [0,0,0]
                image = image1 
                temp1 = get_board(image)
                temp2 = get_chess(image,depth_image,'x')
                cv2.imshow('realsense',image)
                               
                     
        while not rospy.is_shutdown():
    	        rospy.spin()
     
        


'''
#get video
while(True):
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        image = np.asanyarray(color_frame.get_data())
        shape = image.shape
        out.write(image)
        
        # Cut frame
        image[0:upper,0:shape[1]] = [0,0,0]
        image[upper:lower,0:left] = [0,0,0]
        image[upper:lower,right:shape[1]] = [0,0,0]
        image[lower:shape[0],0:shape[1]] = [0,0,0]
        
        cv2.imshow('realsense',image)                            

        if cv2.waitKey(1) & 0xFF == ord('z'):
                print(get_boardpos(image,depth_image))
                print('x:',get_chess(image,depth_image,'x'))  
                print('o:',get_chess(image,depth_image,'o')) 
                if lastboardcase is None: 
                        chessboard = get_board(image)
                diff = 0
                for i in range(3):
                        for j in range(3):
                                diff = diff + abs(chessboard[i][j] - lastboardcase[i][j])       
                if (diff == 1) | (diff == 0) | (diff == 2):
                        print(get_board(image))
                else:
                        print('Invaild step!')
        if  cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
               
out.release()
pipeline.stop()
'''               
                
                
                
