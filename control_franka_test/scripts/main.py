#!/usr/bin/env python
import rospy
import ai
import numpy as np
import time
import control_test
from control_franka_test.srv import *

# TODO define ros client
def get_chess(chess):
    if chess == 'O':
        chess = 1
    else:
        chess = 2
    rospy.wait_for_service('boardcase')
    try:
        boardcase = rospy.ServiceProxy('boardcase', board_detector)
        resp = boardcase(chess)
        return resp.chess_location        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def get_boardcase():
    rospy.wait_for_service('boardcase')
    try:
        boardcase = rospy.ServiceProxy('boardcase', board_detector)
        resp = boardcase(1)
        return resp.boardcase       
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def get_boardpos():
    rospy.wait_for_service('boardcase')
    try:
        boardcase = rospy.ServiceProxy('boardcase', board_detector)
        resp = boardcase(1)
        resp = [resp.pt0, resp.pt1, resp.pt2, resp.pt3, resp.pt4, resp.pt5, resp.pt6, resp.pt7, resp.pt8]
        return resp        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        

# TODO init hand eye matrix
'''
hand_eye_matrix = np.array([[-6.63929732e-04  ,7.25966299e-06  ,6.25425345e-05  ,9.41958124e-01],
                            [ 9.88426913e-06  ,6.63133454e-04 ,-1.15013110e-04  ,4.31614450e-02],
                            [ 3.27028551e-05 ,-1.46691850e-04 ,-8.96244009e-04  ,1.03196302e+00],
                            [ 0.0  ,0.0 ,0.0  ,1.00000000e+00]])

hand_eye_matrix = np.array([[-9.43816127e-04  ,2.67391034e-05  ,4.20246045e-05  ,1.12050524e+00],
 [ 1.83531828e-05  ,9.31285435e-04 ,-1.18476750e-04 ,-7.53148424e-02],
 [-3.29899181e-05 ,-6.71508851e-05 ,-3.00793233e-04  ,4.92525695e-01],
 [-6.02724482e-19 ,-2.16840434e-19  ,9.75781955e-19  ,1.00000000e+00]])
                             '''
hand_eye_matrix = np.array([[-1.01366878e+00  ,2.78444829e-02 , 3.91955428e-02  ,5.46183880e-01],
 [ 1.22528988e-02  ,1.00105239e+00 ,-1.77008793e-01  ,3.19519810e-01],
 [-3.71608277e-02 ,-1.57561013e-01 ,-8.65574784e-01  ,9.46028494e-01],
 [-1.23164781e-15 ,-8.88178420e-16 ,-4.44089210e-16  ,1.00000000e+00]])
 

# TODO init panda
pc = control_test.my_panda()

# TODO set const and origin
x_origin = 0.571507
y_origin = -0.28287
z_high = 0.2599
z_low = 0.089
panda_origin = pc.set_goal([x_origin, y_origin, z_high])

# TODO init chess board
board = ai.Board()

# TODO main loop
while True:

    ans = raw_input('\rPlay game with panda?  [y/n]:')
    if ans == 'n':
        break

    # who first
    done = False
    while not done:
        ans = raw_input('\rwho play first hand? [1.panda/2.human]:')
        if int(ans) is not 1 and int(ans) is not 2:
            print('\rInput 1 or 2 PLEASE!')
        elif int(ans) is 1:
            first_hand = 'panda'
            #print('Panda play first')
            done = True
        elif int(ans) is 2:
            first_hand = 'human'
            #print('human play first, please play X chess')
            done = True
            
    # TODO FIRST HAND PLAY X CHESS
    if first_hand == 'panda':
        panda_chess = 'X'
        human_chess = 'O'
        human_chess_num = 1
        panda_chess_num = 2
    else:
        panda_chess = 'O'
        human_chess = 'X'
        human_chess_num = 2
        panda_chess_num = 1
    # TODO init ai
    ai = ai.tictactoe_ai(panda_chess)

    #game loop
    done = False
    current_player = first_hand
    panda_play_buffer = []
    human_play = []
    board_state = [0] * 9
    while not done:
        if current_player == 'panda':

            # TODO detect the board and chess
            
            #board_state = get_boardcase()
            #board.set_borad(list(board_state))
            #print('board state:', board_state)
            
            chess_location = get_chess(panda_chess)
            chess_location = list(chess_location)
            #print(chess_location)
            #point = cam([chess_location[0], chess_location[1]])#.....................cam
            #chess_location[0] = point[0]
            #chess_location[1] = point[1]
            chess_location = chess_location + [1.0]
            chess_location = np.array(chess_location)  # [x, y, z, 1]
            
            chess_location = hand_eye_matrix.dot(chess_location.T)  # real world

            # TODO think
            action = ai.think(board)
            
            panda_play_buffer = panda_play_buffer + [action]
            #board_state[action] = panda_chess_num
            board_state = list(board_state)
            for i in range(9):
                if i in panda_play_buffer:
                    board_state[i] = panda_chess_num
                elif i in human_play:
                    board_state[i] = human_chess_num
                else:
                    board_state[i] = 0
            board.set_borad(list(board_state))
            print('action:', action, board_state)
            if board.teminate:
                break
            # TODO plan
            place_location = get_boardpos()
            place_location = place_location[action]
            place_location = list(place_location)
            #point = cam([place_location[0], place_location[1]])
            #place_location[0] = point[0]
            #place_location[1] = point[1] - 23
            place_location = np.array(place_location + [1.0])  # [x, y, z, 1]
            place_location = hand_eye_matrix.dot(place_location.T)  # real world

            # TODO move real robot
            # move x y --> move z --> pick --> move z --> move x y --> place --> back to origin
            goal = pc.set_goal([chess_location[0], chess_location[1], z_high])
            pc.move_arm(goal)  # move x y

            goal = pc.set_goal([chess_location[0], chess_location[1], z_low])
            pc.move_arm(goal)  # move z

            pc.close_gripper()  # pick

            goal = pc.set_goal([chess_location[0], chess_location[1], z_high])
            pc.move_arm(goal) # move z

            goal = pc.set_goal([place_location[0], place_location[1], z_high])
            pc.move_arm(goal)  # move x y
            
            goal = pc.set_goal([place_location[0], place_location[1], z_low + 0.01])
            pc.move_arm(goal) # move z
            pc.open_gripper()  # place
            goal = pc.set_goal([place_location[0], place_location[1], z_high])
            pc.move_arm(goal) # move z
            
            goal = pc.set_goal([x_origin, y_origin, z_high])
            pc.move_arm(goal)  # move to origin

            current_player = 'human'  # switch hand
        else:
            # TODO detect finish
            board_state = get_boardcase()
            while board_state == None:
                board_state = get_boardcase()
                pass
            start_boardsum = sum(list(board_state))
            #print('start_boardsum:', start_boardsum)
            my_buff = [-1] * 20
            done1 = False
            
            while not done1:
                board_state = get_boardcase()
                board_state = [a for a in board_state if a == human_chess_num]
                print(board_state)
                boardm = sum(list(board_state)) - start_boardsum
                del my_buff[0]
                my_buff = my_buff + [boardm]
                var = np.mean(my_buff)
                print('var', var)
                if var == human_chess_num:
                    done1 = True
                time.sleep(0.15)
            for i in range(9):
                if board_state[i] == human_chess_num:
                    human_play = human_play + [i]
            current_player = 'panda'  # switch hand

print('-'*9+'bye!'+'-'*9)
