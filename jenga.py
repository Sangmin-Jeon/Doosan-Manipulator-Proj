import rclpy
import DR_init

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import tkinter as tk
from tkinter import StringVar
import threading

from dsr_msgs2.srv import SetRobotMode

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 50, 50
GRIP_VELOCITY, GRIP_ACC = 10, 10

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

'''
    # 1
    l:[508.887, 66.002, 117.368, 90.883, 89.992, -89.985]
    j:[-29.231, 53.419, 81.938, 67.559, 111.09, -229.687]

    --------------------------------------------------------------------

    # right 1
    l: [508.887, 66.002, 90.01, 90.883, 89.992, -89.985]

    # right 2
    l: [508.887, 66.002, 117.368, 90.883, 89.992, -89.985]

    # right 3
    l: [508.887, 66.002, 147.01, 90.883, 89.992, -89.985]

    # right 4
    l: [508.887, 66.002, 177.01, 90.883, 89.992, -89.985]

    # right 5
    l: [508.887, 66.002, 206.01, 90.883, 89.992, -89.985]

    # right 6
    l: [508.887, 66.002, 235.01, 90.883, 89.992, -89.985]

    --------------------------------------------------------------------

    # left 1
    - SKIP - 

    # left 2
    l: [507.65, 96.7, 114.82, 88.47, -90.0, -90.0]

    # left 3
    l: [507.65, 96.7, 144.82, 88.47, -90.0, -90.0]

    # left 4
    l: [507.65, 96.7, 174.82, 88.47, -90.0, -90.0]

    # left 5
    l: [507.65, 96.7, 204.82, 88.47, -90.0, -90.0]

    # left 6
    l: [507.65, 96.7, 232.82, 88.47, -90.0, -90.0]

    --------------------------------------------------------------------

    # 3
    l:[508.88, 65.01, 267.31, 90.88, 89.99, -89.98]


    --------------------------------------------------------------------
    drop pose
    # 1
    l: [508.88, 107.0, 267.31, 90.88, 89.99, -89.98]

    # 2
    l: [508.88, 82.0, 267.31, 90.88, 89.99, -89.98]

    # 3
    l: [508.88, 60.5, 267.31, 90.88, 89.99, -89.98]

    --------------------------------------------------------------------
    middle pose
    # 1
    l: [500.0, 160.0, 320.0, 95.0, -160.0, -75.0]

    # 2
    l: [500.0, 20.0, 340.0, 95.0, 95.0, -90.0]
'''


JReady = [-4, -31, 72, 5, 51, -230]    
    
def get_right_list():
    right1 = [508.887, 66.002, 90.01, 90.883, 89.992, -89.985]

    # right 2
    right2 = [508.887, 66.002, 117.368, 90.883, 89.992, -89.985]

    # right 3
    right3 = [508.887, 66.002, 147.01, 90.883, 89.992, -89.985]

    # right 4
    right4 = [508.887, 66.002, 177.01, 90.883, 89.992, -89.985]

    # right 5
    right5 = [508.887, 66.002, 206.01, 90.883, 89.992, -89.985]

    # right 6
    right6 = [508.887, 66.002, 235.01, 90.883, 89.992, -89.985]

    return [right1, right2, right3, right4, right5, right6]

def get_left_list():
    # left 2
    left2 = [507.65, 96.7, 114.82, 88.47, -90.0, -90.0]

    # left 3
    left3 = [507.65, 96.7, 144.82, 88.47, -90.0, -90.0]

    # left 4
    left4 = [507.65, 96.7, 174.82, 88.47, -90.0, -90.0]

    # left 5
    left5 = [507.65, 96.7, 204.82, 88.47, -90.0, -90.0]

    # left 6
    left6 = [507.65, 96.7, 232.82, 88.47, -90.0, -90.0]

    return [left2, left2, left3, left4, left5, left6]

# 1
via_pose1 = [500.0, 160.0, 320.0, 95.0, -160.0, -75.0]

# 2
via_pose2 = [500.0, 20.0, 340.0, 95.0, 95.0, -90.0]

drop_init_pose = [508.88, 29.18, 267.31, 90.88, 89.99, -89.98]

drop_pose1 = [508.88, 112.0, 267.31, 90.88, 89.99, -89.98]

# 2
drop_pose2 = [508.88, 87.0, 267.31, 90.88, 89.99, -89.98]

# 3
drop_pose3 = [508.88, 65.5, 267.31, 90.88, 89.99, -89.98]


drop_init_pose_second = [508.88, 29.18, 282.31, 90.88, 89.99, -89.98]

drop_pose1_second = [508.88, 112.0, 282.31, 90.88, 89.99, -89.98]

# 2
drop_pose2_second = [508.88, 87.0, 282.31, 90.88, 89.99, -89.98]

# 3
drop_pose3_second = [508.88, 65.5, 282.31, 90.88, 89.99, -89.98]
spin_floor = [510.78, 79.1, 261.39, 27.82, 180.0, 115.9] 
spin_grip = [510.78, 79.1, 253.39, 27.82, 180.0, 115.9] 


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            trans,
            get_current_posx,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_MOD_REL,
            
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    

    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)


    def move_back_pose(coordinate, is_left):
        lf_delta = posx([0, 30, 0, 0, 0, 0])
        rt_delta = posx([0, -30, 0, 0, 0, 0])

        place = posx(coordinate) 
        
        if is_left:
            tr_safe_left = trans(place, lf_delta, ref_out=DR_BASE)
            movel(tr_safe_left, vel=GRIP_VELOCITY, acc=GRIP_ACC)
        else:
            tr_safe_right = trans(place, rt_delta, ref_out=DR_BASE)
            movel(tr_safe_right, vel=GRIP_VELOCITY, acc=GRIP_ACC)

    def right_grip(right_pose, order):
        # 잡으려는 위치로 이동
        movel(posx(right_pose), vel=VELOCITY, acc=ACC)  
        grip()

        # 잡고 뒤로 살짝 이동
        move_back_pose(right_pose, False)

        # 놓을 위치로 이동
        movel(posx(drop_init_pose), vel=VELOCITY, acc=ACC)
        movel(posx(order), vel=VELOCITY, acc=ACC)

        release()
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)

    def left_grip(left_pose, order):
        # 반대편으로 이동
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)
        movel(posx(via_pose1), vel=VELOCITY, acc=ACC)

        # 왼쪽 물제 잡는 포즈 이동
        movel(posx(left_pose), vel=VELOCITY, acc=ACC)
        grip()
        # 잡고 뒤로 살짝 이동
        move_back_pose(left_pose, True)

        # 반대편으로 이동
        movel(posx(via_pose1), vel=VELOCITY, acc=ACC)
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)

        # 놓을 위치로 이동
        movel(posx(drop_init_pose), vel=VELOCITY, acc=ACC)
        movel(posx(order), vel=VELOCITY, acc=ACC)
        release()
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)
        
    def right_grip_second(right_pose, order):
        # 잡으려는 위치로 이동
        movel(posx(right_pose), vel=VELOCITY, acc=ACC)  
        grip()

        # 잡고 뒤로 살짝 이동
        move_back_pose(right_pose, False)

        # 놓을 위치로 이동
        movel(posx(drop_init_pose_second), vel=VELOCITY, acc=ACC)
        movel(order, vel=VELOCITY, acc=ACC)

        release()
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)

    def left_grip_second(left_pose, order):
        # 반대편으로 이동
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)
        movel(posx(via_pose1), vel=VELOCITY, acc=ACC)

        # 왼쪽 물제 잡는 포즈 이동
        movel(posx(left_pose), vel=VELOCITY, acc=ACC)
        grip()
        # 잡고 뒤로 살짝 이동
        move_back_pose(left_pose, True)

        # 반대편으로 이동
        movel(posx(via_pose1), vel=VELOCITY, acc=ACC)
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)

        # 놓을 위치로 이동
        movel(posx(drop_init_pose_second), vel=VELOCITY, acc=ACC)
        movel(order, vel=VELOCITY, acc=ACC)
        release()
        movel(posx(via_pose2), vel=VELOCITY, acc=ACC)
        
    def start_jenga():
        # init pose
        release()
        movej(posj(JReady), vel=30, acc=30)  
        print('jenga start')

        left_list = get_left_list()
        right_list = get_right_list()
        while True:  
                first_pose = input("첫번째 위치를 입력: ")


                if first_pose.startswith("left"):
                        print(f"'{first_pose}'는 'left'로 시작합니다.")
                        pose = left_list[int(first_pose.strip()[-1]) - 1]
                        left_grip(pose, drop_pose1)
                        break  

                elif first_pose.startswith("right"):
                        print(f"'{first_pose}'는 'right'로 시작합니다.")
                        pose = right_list[int(first_pose.strip()[-1]) - 1]
                        right_grip(pose, drop_pose1)
                        break 

                else:
                        print(f"'{first_pose}'는 'left'나 'right'로 시작하지 않습니다. 다시 입력해주세요.")

        while True:  
                second_pose = input("두번째 위치를 입력: ")
                

                if second_pose.startswith("left"):
                    pose = left_list[int(second_pose.strip()[-1]) - 1]
                    left_grip(pose, drop_pose2)
                    break 

                elif second_pose.startswith("right"):
                    print(f"'{second_pose}'는 'right'로 시작합니다.")
                    pose = right_list[int(second_pose.strip()[-1]) - 1]
                    right_grip(pose, drop_pose2)
                    break  

                else:
                    print(f"'{second_pose}'는 'left'나 'right'로 시작하지 않습니다. 다시 입력해주세요.")
                    
        while True:  
                third_pose = input("세번째 위치를 입력: ")
                

                if third_pose.startswith("left"):
                    print(f"{third_pose}는 'left'로 시작합니다.")
                    pose = left_list[int(third_pose.strip()[-1]) - 1]
                    left_grip(pose, drop_pose3)
                    break  

                elif third_pose.startswith("right"):
                    print(f"'{third_pose}'는 'right'로 시작합니다.")
                    pose = right_list[int(third_pose.strip()[-1]) - 1]
                    right_grip(pose, drop_pose3)
                    break 

                else:
                    print(f"'{third_pose}'는 'left'나 'right'로 시작하지 않습니다. 다시 입력해주세요.")
        # 두번째층
        delta_second_floor = posx([0,0,15,0,0,0])
        drop_pose_second1 = trans(drop_pose1, delta_second_floor, ref_out=DR_BASE)    
        drop_pose_second2 = trans(drop_pose2, delta_second_floor, ref_out=DR_BASE)  
        drop_pose_second3 = trans(drop_pose3, delta_second_floor, ref_out=DR_BASE)         
        while True:  
                first_pose = input("첫번째 위치를 입력: ")


                if first_pose.startswith("left"):
                        print(f"'{first_pose}'는 'left'로 시작합니다.")
                        pose = left_list[int(first_pose.strip()[-1]) - 1]
                        left_grip_second(pose, drop_pose_second1)
                        break  

                elif first_pose.startswith("right"):
                        print(f"'{first_pose}'는 'right'로 시작합니다.")
                        pose = right_list[int(first_pose.strip()[-1]) - 1]
                        right_grip_second(pose, drop_pose_second1)
                        break 

                else:
                        print(f"'{first_pose}'는 'left'나 'right'로 시작하지 않습니다. 다시 입력해주세요.")

        while True:  
                second_pose = input("두번째 위치를 입력: ")
                

                if second_pose.startswith("left"):
                    pose = left_list[int(second_pose.strip()[-1]) - 1]
                    left_grip_second(pose, drop_pose_second2)
                    break 

                elif second_pose.startswith("right"):
                    print(f"'{second_pose}'는 'right'로 시작합니다.")
                    pose = right_list[int(second_pose.strip()[-1]) - 1]
                    right_grip_second(pose, drop_pose_second2)
                    break  

                else:
                    print(f"'{second_pose}'는 'left'나 'right'로 시작하지 않습니다. 다시 입력해주세요.")
                    
        while True:  
                third_pose = input("세번째 위치를 입력: ")
                

                if third_pose.startswith("left"):
                    print(f"{third_pose}는 'left'로 시작합니다.")
                    pose = left_list[int(third_pose.strip()[-1]) - 1]
                    left_grip_second(pose, drop_pose_second3)
                    break  

                elif third_pose.startswith("right"):
                    print(f"'{third_pose}'는 'right'로 시작합니다.")
                    pose = right_list[int(third_pose.strip()[-1]) - 1]
                    right_grip_second(pose, drop_pose_second3)
                    break 

                else:
                    print(f"'{third_pose}'는 'left'나 'right'로 시작하지 않습니다. 다시 입력해주세요.")
                    
        movej(posj(JReady), vel=30, acc=30)  
        movel(posx(spin_floor), vel=VELOCITY, acc=ACC)
        movel(posx(spin_grip), vel=VELOCITY, acc=ACC)
        grip()
        movel(posx(spin_floor), vel=VELOCITY, acc=ACC)
        spin_90 = trans(posx(spin_floor), posx([0,0,0,0,0,90]), ref_out=DR_BASE)
        spin_rs = trans(posx(spin_grip), posx([0,0,0,0,0,90]), ref_out=DR_BASE)
        movel(spin_90, vel=VELOCITY, acc=ACC)
        movel(spin_rs, vel=VELOCITY, acc=ACC)
        release()


    ''' 시작 점'''
    start_jenga()
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
