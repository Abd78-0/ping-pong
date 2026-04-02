import cv2
import numpy as np
import time
import math
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
font = cv2.FONT_HERSHEY_SIMPLEX
label = "orange"

from dynamixel_sdk import *  # Dynamixel SDK
 
# === Constants ===
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
DEVICENAME = '/dev/ttyUSB0'  # Change if needed
BAUDRATE = 57600
MOTOR_IDS = [0, 3, 6, 9]
LEVEL_POS = "0 5 5 5"
CENTREZONE_POS = "3 8 8 8"		# All Servos add 3
INNERZONE1_POS = "8 10 8 10" 	# Servo 0 add 5. Servos 3 and 9 add 2
INNERZONE2_POS = "8 13 8 8" 	# Servos 0 and 3 add 5.
INNERZONE3_POS = "5 13 10 8" 	# Servo 3 add 5. Servos 0 and 6 add 2
INNERZONE4_POS = "3 13 13 8"	# Servos 3 and 6 add 5.
INNERZONE5_POS = "3 10 13 10"	# Servos 6 add 5. Servos 3 and 9 add 2
INNERZONE6_POS = "3 8 13 13"	# Servos 6 and 9 add 5.
INNERZONE7_POS = "5 8 10 13" 	# Servo 9 add 5. Servos 0 and 6 add 2
INNERZONE8_POS = "8 8 8 13"		# Servos 0 and 9 add 5
MIDDLEZONE1_POS = "18 14 8 14" 	# Servo 0 add 15. Servos 3 and 9 add 6
MIDDLEZONE2_POS = "18 23 8 8" 	# Servos 0 and 3 add 15.
MIDDLEZONE3_POS = "6 23 14 8" 	# Servo 3 add 15. Servos 0 and 6 add 6
MIDDLEZONE4_POS = "3 23 23 8"	# Servos 3 and 6 add 15.
MIDDLEZONE5_POS = "3 14 23 14"	# Servos 6 add 15. Servos 3 and 9 add 6
MIDDLEZONE6_POS = "3 8 23 23"	# Servos 6 and 9 add 15.
MIDDLEZONE7_POS = "9 8 14 23" 	# Servo 9 add 15. Servos 0 and 6 add 6
MIDDLEZONE8_POS = "18 8 8 23"	# Servos 0 and 9 add 15
OUTERZONE1_POS = "18 16 8 16" 	# Servo 0 add 15. Servos 3 and 9 add 8
OUTERZONE2_POS = "18 23 8 8" 	# Servos 0 and 3 add 15.
OUTERZONE3_POS = "11 23 16 8" 	# Servo 3 add 15. Servos 0 and 6 add 8
OUTERZONE4_POS = "3 23 23 8"	# Servos 3 and 6 add 15.
OUTERZONE5_POS = "3 16 23 16"	# Servos 6 add 15. Servos 3 and 9 add 8
OUTERZONE6_POS = "3 8 23 23"	# Servos 6 and 9 add 15.
OUTERZONE7_POS = "11 8 16 23" 	# Servo 9 add 15. Servos 0 and 6 add 8
OUTERZONE8_POS = "18 8 8 23"	# Servos 0 and 9 add 15

# Control table address for present position (Protocol 1.0)
ADDR_PRESENT_POSITION = 36

# === Flags ===
bReact = True
bDebug = False
 
# === Initialize Port and Packet Handlers ===
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, 2)  # 2 bytes
 
# === Open Port and Set Baudrate ===
if not portHandler.openPort():
    raise Exception("❌ Failed to open port")
 
if not portHandler.setBaudRate(BAUDRATE):
    raise Exception("❌ Failed to set baudrate")
 
print("✅ Port opened and baudrate set")
print("Controlling motors:", MOTOR_IDS)


ADDR_MX_MOVING_SPEED = 32           # Control Table 地址
desired_speed = 300                 # 范围 0~2047，0=最大速，1~1023=CCW 方向，1024~2047=CW 方向

# # 给 舵机写入速度
# dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
#     portHandler, 0, ADDR_MX_MOVING_SPEED, desired_speed
# )

# dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
#     portHandler, 3, ADDR_MX_MOVING_SPEED, desired_speed
# )

# dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
#     portHandler, 6, ADDR_MX_MOVING_SPEED, desired_speed
# )

# dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
#     portHandler, 9, ADDR_MX_MOVING_SPEED, desired_speed
# )

# 初始化串口、packetHandler 之后
SERVO_SPEED = 1023
max_speed = 480

ADDR_MX_MOVING_SPEED = 32   # Control Table 地址

def set_moving_speed(motor_id: int, speed: int):
    dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
        portHandler,
        motor_id,
        ADDR_MX_MOVING_SPEED,
        speed
    )
    if dxl_comm != COMM_SUCCESS:
        print(f"❌ Motor {motor_id} speed set failed: {packetHandler.getTxRxResult(dxl_comm)}")
    elif dxl_err != 0:
        print(f"❌ Motor {motor_id} reported error: {packetHandler.getRxPacketError(dxl_err)}")
    else:
        print(f"✅ Motor {motor_id} speed set to {speed}")


# 初始化串口、packetHandler 之后
SERVO_SPEED = 1023
max_speed = 480

set_moving_speed(0, SERVO_SPEED)
# set_moving_speed(3, 300)
set_moving_speed(3, SERVO_SPEED)
# set_moving_speed(6, 330)
set_moving_speed(6, SERVO_SPEED)  
set_moving_speed(9, SERVO_SPEED)

  
# === Angle Conversion Function (per motor model) ===
def angle_to_dxl(motor_id, angle_deg):

    max_deg = 250
    resolution = 0.06
 
    angle_deg = max(0, min(max_deg, angle_deg))  # Clamp angle
    return int(angle_deg / resolution)



def motorMove(value_input):
    angle_vals = list(map(float, value_input.strip().split()))
    if len(angle_vals) != 4:
        print("⚠️ Please enter 4 angles.")
        
 
    # Clear previous params
    groupSyncWrite.clearParam()
 
    # dc_list = [0, 0, 0, 0]
    # count = 0 
 
    for motor_id, angle in zip(MOTOR_IDS, angle_vals):
        dxl_position = angle_to_dxl(motor_id, angle)
        # dxl_present_pos, dxl_comm_res, dxl_err = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
        # dc_list[count] = abs(dxl_position - dxl_present_pos)
        # count = count + 1
        param_goal_position = [DXL_LOBYTE(dxl_position), DXL_HIBYTE(dxl_position)]
        success = groupSyncWrite.addParam(motor_id, param_goal_position)
        if not success:
            print(f"❌ Failed to add param for motor {motor_id}")
 
    # max_dc = 0
    # for item in dc_list:
    #    if item > max_dc:
    #        max_dc = item
    # for count in range(len(dc_list)):
    #     if dc_list[count] != 0:
    #        dxl_comm, dxl_err = packetHandler.write2ByteTxRx(portHandler, count*3, ADDR_MX_MOVING_SPEED, int(max_speed*dc_list[count]/max_dc))
 
    # Send sync write packet
    dxl_comm_result = groupSyncWrite.txPacket()






 

def main():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, 30)
    print("驱动返回 FPS =", cap.get(cv2.CAP_PROP_FPS))

    r_target    = 57         # 触板最大半径
    r_threshold = 26         # 触发舵机阈值

    recorded_fall_point = False
    prev_center = prev_radius = prev_time = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧，退出程序")
            break

        # cv2.imshow("Image", frame)
        results = model(frame)
        res = results[0]
        for i in range(len(res.boxes)):
            cls_idx = int(res.boxes.cls[i])
            name = res.names[cls_idx]
            if name == label:
                x1,y1,x2,y2 = map(int,res.boxes.xyxy[i])
                x, y, r = x1,y1, (((x1-x2) + (y1-y2))//4)*(-1)

                if r > 3:
                    center = (int(x), int(y))
                    cv2.circle(frame, center, r, (0, 255, 255), 2)
                    cv2.circle(frame, center, 2, (0, 0, 255), -1)

                    # -------- 仅当 prev_* 已存在才做运算 --------
                    if prev_center is not None:
                        cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                        x_pred = cx + (x - cx) / r * r_target
                        y_pred = cy + (y - cy) / r * r_target
                        
                        # Calculate the angle of the center point
                        deltaX = x_pred - 320
                        deltaY = y_pred - 240
                        pt_angle = math.atan2(deltaX,-deltaY)

                        if r > r_threshold:
                            if r < prev_radius:                          # 正常阶段
                                if bDebug:
                                    print(f"Centre of the ball:({center[0]}, {center[1]}), r={r:.1f} → Predicted falling point:({x_pred:.1f}, {y_pred:.1f})")
                                # user_input = "0 5 0 0"
                                # user_input = "0 0 0 0"
                                user_input = LEVEL_POS
                                motorMove(user_input)

                            elif r > prev_radius:                # 到阈值首次触发
                                recorded_fall_point = True
                                if bDebug:
                                    print(f"★★ 触发舵机 | r={r:.1f} ≥ {r_threshold} | 落点({x_pred:.1f},{y_pred:.1f}) ★★")
                                if (((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5) < 50:
                                    # Centre
                                    if bDebug:
                                        print("Centre")
                                    if bReact == True:
                                        user_input = CENTREZONE_POS
                                        motorMove(user_input)
                                elif (((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5) < 240:
                                    # Inner Ring
                                    if bDebug:
                                        print("Inner Ring")
                                    if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                        ## Inner Zone 1 ##
                                        if bDebug:
                                            print("Inner Zone 1")
                                        if bReact == True:
                                            user_input = INNERZONE1_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
                                        ## Inner Zone 2 ##
                                        if bDebug:
                                            print("Inner Zone 2")
                                        if bReact == True:
                                            user_input = INNERZONE2_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
                                        ## Inner Zone 3 ##
                                        if bDebug:
                                            print("Inner Zone 3")
                                        if bReact == True:
                                            user_input = INNERZONE3_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
                                        ## Inner Zone 4 ##
                                        if bDebug:
                                            print("Inner Zone 4")
                                        if bReact == True:
                                            user_input = INNERZONE4_POS
                                            motorMove(user_input)
                                    elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
                                        ## Inner Zone 5 ##
                                        if bDebug:
                                            print("Inner Zone 5")
                                        if bReact == True:
                                            user_input = INNERZONE5_POS
                                            motorMove(user_input)
                                    elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
                                        ## Inner Zone 6 ##
                                        if bDebug:
                                            print("Inner Zone 6")
                                        if bReact == True:
                                            user_input = INNERZONE6_POS
                                            motorMove(user_input)
                                    elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
                                        ## Inner Zone 7 ##
                                        if bDebug:
                                            print("Inner Zone 7")
                                        if bReact == True:
                                            user_input = INNERZONE7_POS
                                            motorMove(user_input)
                                    else:
                                        ## Inner Zone 8 ##
                                        if bDebug:
                                            print("Inner Zone 8")
                                        if bReact == True:
                                            user_input = INNERZONE8_POS
                                            motorMove(user_input)
                                    
                                elif (((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5) < 480:
                                    # Middle Ring
                                    if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                        ## Middle Zone 1 ##
                                        if bDebug:
                                            print("Middle Zone 1")
                                        if bReact == True:
                                            user_input = MIDDLEZONE1_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
                                        ## Middle Zone 2 ##
                                        if bDebug:
                                            print("Middle Zone 2")
                                        if bReact == True:
                                            user_input = MIDDLEZONE2_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
                                        ## Middle Zone 3 ##
                                        if bDebug:
                                            print("Middle Zone 3")
                                        if bReact == True:
                                            user_input = MIDDLEZONE3_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
                                        ## Middle Zone 4 ##
                                        if bDebug:
                                            print("Middle Zone 4")
                                        if bReact == True:
                                            user_input = MIDDLEZONE4_POS
                                            motorMove(user_input)
                                    elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
                                        ## Middle Zone 5 ##
                                        if bDebug:
                                            print("Middle Zone 5")
                                        if bReact == True:
                                            user_input = MIDDLEZONE5_POS
                                            motorMove(user_input)
                                    elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
                                        ## Middle Zone 6 ##
                                        if bDebug:
                                            print("Middle Zone 6")
                                        if bReact == True:
                                            user_input = MIDDLEZONE6_POS
                                            motorMove(user_input)
                                    elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
                                        ## Middle Zone 7 ##
                                        if bDebug:
                                            print("Middle Zone 7")
                                        if bReact == True:
                                            user_input = MIDDLEZONE7_POS
                                            motorMove(user_input)
                                    else:
                                        ## Middle Zone 8 ##
                                        if bDebug:
                                            print("Middle Zone 8")
                                        if bReact == True:
                                            user_input = MIDDLEZONE8_POS
                                            motorMove(user_input)
                                else:
                                    # Outer Ring
                                    if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                        ## Outer Zone 1 ##
                                        if bDebug:
                                            print("Outer Zone 1")
                                        if bReact == True:
                                            user_input = OUTERZONE1_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
                                        ## Outer Zone 2 ##
                                        if bDebug:
                                            print("Outer Zone 2")
                                        if bReact == True:
                                            user_input = OUTERZONE2_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
                                        ## Outer Zone 3 ##
                                        if bDebug:
                                            print("Outer Zone 3")
                                        if bReact == True:
                                            user_input = OUTERZONE3_POS
                                            motorMove(user_input)
                                    elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
                                        ## Outer Zone 4 ##
                                        if bDebug:
                                            print("Outer Zone 4")
                                        if bReact == True:
                                            user_input = OUTERZONE4_POS
                                            motorMove(user_input)
                                    elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
                                        ## Outer Zone 5 ##
                                        if bDebug:
                                            print("Outer Zone 5")
                                        if bReact == True:
                                            user_input = OUTERZONE5_POS
                                            motorMove(user_input)
                                    elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
                                        ## Outer Zone 6 ##
                                        if bDebug:
                                            print("Outer Zone 6")
                                        if bReact == True:
                                            user_input = OUTERZONE6_POS
                                            motorMove(user_input)
                                    elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
                                        ## Outer Zone 7 ##
                                        if bDebug:
                                            print("Outer Zone 7")
                                        if bReact == True:
                                            user_input = OUTERZONE7_POS
                                            motorMove(user_input)
                                    else:
                                        ## Outer Zone 8 ##
                                        if bDebug:
                                            print("Outer Zone 8")
                                        if bReact == True:
                                            user_input = OUTERZONE8_POS
                                            motorMove(user_input)
                                

                        
                    # -------- 更新上一帧 --------
                    prev_center, prev_radius, prev_time = center, r, time.time()
            
            else:
                # user_input = "0 5 0 0"
                user_input = LEVEL_POS
                motorMove(user_input)
                    

            # cv2.imshow("Camera Upward View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        # user_input = "0 5 0 0"
        user_input = LEVEL_POS
        motorMove(user_input)


if __name__ == "__main__":
    main()