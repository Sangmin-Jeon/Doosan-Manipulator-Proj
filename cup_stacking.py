'''
잡는 위치 : [418.073, -189.616, 209.638, 126.688, 179.967, 110.777]
잡는 위치 : [418.073, -189.616, 209.638-11, 126.688, 179.967, 110.777]

놓을 위치(1) : [553.059, 229.599, 85.371, 52.043, 179.935, 36.107]
놓을 위치(2) : [553.059, 229.599-81, 85.371, 52.043, 179.935, 36.107]

회전 잡는 위치(J) : [-48.677, 53.916, 84.986, 55.382, 116.717, -57.567]
회전 잡는 위치(X) : [419.098, -80.105, 99.038, 83.707, 89.977, 89.953]

최종 위치 : [500.259, 180.352, 306.201, 94.586, 89.994, -89.983]
'''

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 300, 300
J_VELOCITY, J_ACC = 35, 35

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0
       
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
            movej,
            movel,
            wait,
            amovel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
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
    
    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    
    # 회전할 컵 잡는 위치 movej
    TurnJ = [-48.677, 53.916, 84.986, 55.382, 116.717, -57.567]     
    
    place10 = posx([553.059-140+46.2+46.2, 229.599-40-40, 345.371, 52.043, 179.935, 36.107])    
    place11 = posx([500.259, 178.352, 313.201, 94.586, 89.994, -89.983])    
    
    moveup = posx([0, 0, 23, 0, 0, 0])
    movedown = posx([0, 0, -29, 0, 0, 0])
    
    moveside = posx([0, 200, 0, 0, 0, 0])
    placedown = posx([0, 0, -72, 0, 0, 0])
    
    moveup2 = posx([0, 0, 27, 0, 0, 0])
    movedown2 = posx([0, 0, -13, 0, 0, 0])
    
    pickup = posx([0, 0, 140, 0, 0, 0])
    pickup2 = posx([0, 0, 270, 0, 0, 0])
    
    back = posx([0, -50, 0 , 0, 0, 0])
    
    def start_move():
    # 컵을 잡는 위치
        base_position = [418.073, -109.616, 209.638, 126.688, 179.967, 110.777]
        steps = 11  # 각 단계의 Z 좌표 감소 값
        count = 10  # 생성할 pick 개수
        picks = [
            posx([base_position[0], base_position[1], base_position[2] - steps * i] + base_position[3:])
            for i in range(count)
        ]
        return picks
    
    def finish_move():
        # 정삼각형 변의 길이, 중심 위치 이용하여 계산
        #1단
        place1 = posx([553.059, 229.599, 165.371, 52.043, 179.935, 36.107])
        place2 = posx([553.059, 229.599-81, 165.371, 52.043, 179.935, 36.107])
        place3 = posx([553.059, 229.599-162, 165.371, 52.043, 179.935, 36.107])
        place4 = posx([553.059-70, 229.599-40, 180.371, 52.043, 179.935, 36.107])
        place5 = posx([553.059-70, 229.599-40-81, 165.371, 52.043, 179.935, 36.107])
        place6 = posx([553.059-140, 229.599-40-40, 165.371, 52.043, 179.935, 36.107])
        #2단
        place7 = posx([553.059-70+46.2, 229.599-40, 257.371, 52.043, 179.935, 36.107])
        place8 = posx([553.059-70+46.2, 229.599-40-81, 257.371, 52.043, 179.935, 36.107])
        place9 = posx([553.059-140+46.2, 229.599-40-40, 257.371, 52.043, 179.935, 36.107])
        #3단
        place10 = posx([553.059-140+46.2+46.2, 229.599-40-40, 357.371, 52.043, 179.935, 36.107])
        return [place1, place2, place3, place4, place5, place6, place7, place8, place9, place10]
               
    def turn():
        # 회전
        # 잡는 위치로 이동할 때 간섭을 주지 않기 위해 위치 조정
        movel(place10, vel=VELOCITY, acc=ACC)
        
        # 잡는 위치 movej (옆에서 다가감)
        movej(TurnJ, vel=J_VELOCITY, acc=J_ACC)
        movel(movedown2, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        grip()
        
        # 탑을 쓰러뜨리지 않기 위해 위로 살짝 움직임
        movel(pickup2, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        
        # 회전하여 놓는 위치
        movel(place11, vel=VELOCITY, acc=ACC)
        
        # 힘 제어로 내려가서 놓음
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
                pass
        release_compliance_ctrl()
        release()
               
    # 로봇 동작
    release()
    
    # 초기 위치로 이동
    movej(JReady, vel=J_VELOCITY, acc=J_ACC)
    
    # 1, 2, 3단 쌓기
    # 저장된 잡는 위치 리스트, 놓는 위치 리스트에서 값을 불러와서 차례대로 이동
    for s_pose, f_pose in zip(start_move(), finish_move()):
        release()
        movel(s_pose, vel=VELOCITY, acc=ACC)
        
        # 힘 제어를 이용하여 컵의 위치 확인
        grip()
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=10):
                pass
        release_compliance_ctrl()
        
        # 위치 확인 후 컵 잡기
        movel(moveup, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        release()
        movel(movedown, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL) 
        grip()
        
        # 컵을 이동에 간섭받지 않는 위치로 옮기기 
        amovel(pickup, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        wait(0.9)
        
        # amovel을 이용하여 비동기적으로 이동
        amovel(moveside, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL) 
        amovel(f_pose, vel=VELOCITY, acc=ACC)
        movel(placedown, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        
        # 힘 제어를 이용하여 컵 놓기
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=10):
                pass
        release_compliance_ctrl()
        release()
        
        # 다음 이동에 간섭받지 않도록 위로 살짝 움직이기
        movel(moveup2, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    
    # 마지막 컵 회전하여 쌓기
    turn()
    
    # 초기 위치로 이동
    movel(back, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    
    rclpy.shutdown()
if __name__ == "__main__":
    main()