import cv2
import numpy as np
import time


from dynamixel_sdk import *  # Dynamixel SDK
 
# === Constants ===
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
DEVICENAME = '/dev/ttyUSB0'  # Change if needed
BAUDRATE = 57600
MOTOR_IDS = [0, 3, 6, 9]
 
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


# ADDR_MX_MOVING_SPEED = 32           # Control Table 地址
# desired_speed = 300                 # 范围 0~2047，0=最大速，1~1023=CCW 方向，1024~2047=CW 方向

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



ADDR_MX_MOVING_SPEED = 32  # Control Table 地址

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
set_moving_speed(0, 370)
set_moving_speed(3, 350)  
set_moving_speed(6, 380)  
set_moving_speed(9, 350)  




# === Angle Conversion Function (per motor model) ===
def angle_to_dxl(motor_id, angle_deg):
    if motor_id == 3:
        # MX-106: 250°, 0.088°/step
        max_deg = 250
        resolution = 0.088
    else:
        # EX-106+: 250°, 0.06°/step
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
 
    for motor_id, angle in zip(MOTOR_IDS, angle_vals):
        dxl_position = angle_to_dxl(motor_id, angle)
        param_goal_position = [DXL_LOBYTE(dxl_position), DXL_HIBYTE(dxl_position)]
        success = groupSyncWrite.addParam(motor_id, param_goal_position)
        if not success:
            print(f"❌ Failed to add param for motor {motor_id}")
 
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
    cap.set(cv2.CAP_PROP_FPS, 60)
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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 74, 215]), np.array([18, 255, 255]))
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)



        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            (x, y), r = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
            r = int(r)
            if r > 3:
                center = (int(x), int(y))
                cv2.circle(frame, center, int(r), (0, 255, 255), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), -1)

                # -------- 仅当 prev_* 已存在才做运算 --------
                if prev_center is not None:
                    cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                    x_pred = cx + (x - cx) / r * r_target
                    y_pred = cy + (y - cy) / r * r_target

                    if r > r_threshold:
                        if r < prev_radius:                          # 正常阶段
                            print(f"Centre of the ball:({center[0]}, {center[1]}), r={r:.1f} → Predicted falling point:({x_pred:.1f}, {y_pred:.1f})")
                            user_input = "0 5 0 0"
                            motorMove(user_input)

                        elif r > prev_radius:                # 到阈值首次触发
                            recorded_fall_point = True
                            print(f"★★ 触发舵机 | r={r:.1f} ≥ {r_threshold} | 落点({x_pred:.1f},{y_pred:.1f}) ★★")
                            if (((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5) < 150:
                                print("内侧区域")
                                if (x_pred < 320) and (y_pred < 240):
                                    print ("内部一区：舵机0 舵机3")
                                    user_input = "8 13 5 5"
                                    motorMove(user_input)
                                
                                elif (x_pred < 320) and (y_pred > 240):
                                    print ("内部二区：舵机3 舵机6")
                                    user_input = "5 13 8 5"
                                    motorMove(user_input)

                                elif (x_pred > 320) and (y_pred > 240):
                                    print ("内部三区：舵机6 舵机9")
                                    user_input = "5 10 8 8"
                                    motorMove(user_input)

                                elif (x_pred > 320) and (y_pred < 240):
                                    print ("内部四区：舵机9 舵机0")
                                    user_input = "8 10 5 8"
                                    motorMove(user_input)
                                

                            elif (x_pred < 213) and (y_pred < 133):
                                print("区域一：舵机0 舵机3")
                                user_input = "15 20 0 0"
                                motorMove(user_input)

                            elif (x_pred < 213) and (y_pred > 347):
                                print("区域二：舵机3 舵机6")
                                user_input = "0 23 18 0"
                                motorMove(user_input)

                            elif (x_pred > 418) and (y_pred > 347):
                                print("区域三：舵机6 舵机9")
                                user_input = "0 5 18 18"
                                motorMove(user_input)
                            
                            elif (x_pred > 418) and (y_pred < 133):
                                print("区域四：舵机9 舵机0")
                                user_input = "15 5 0 15"
                                motorMove(user_input)

                            elif (x_pred < 213) and (133 < y_pred < 347):
                                print("三点区域：舵机0 舵机3 舵机6")
                                user_input = "0 20 0 0"
                                motorMove(user_input)

                            elif (213 < x_pred < 418) and (y_pred > 347):
                                print("六点区域：舵机3 舵机6 舵机9")
                                user_input = "0 5 20 0"
                                motorMove(user_input)

                            elif (x_pred > 418) and (133 < y_pred < 347):
                                print("九点区域：舵机6 舵机9 舵机0")
                                user_input = "0 5 0 15"
                                motorMove(user_input)

                            elif (213 < x_pred < 418) and (y_pred < 133):
                                print("零点区域：舵机9 舵机0 舵机6")
                                user_input = "15 5 0 0"
                                motorMove(user_input)

                            

                    
                # -------- 更新上一帧 --------
                prev_center, prev_radius, prev_time = center, r, time.time()
        
        else:
            user_input = "0 5 0 0"
            motorMove(user_input)
                

        #cv2.imshow("Camera Upward View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    user_input = "0 5 0 0"
    motorMove(user_input)


if __name__ == "__main__":
    main()
