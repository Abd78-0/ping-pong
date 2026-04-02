import cv2
import numpy as np
import time
import math

from dynamixel_sdk import *  # Dynamixel SDK
 
# === Constants ===
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
DEVICENAME = '/dev/ttyUSB0'  # Change if needed
BAUDRATE = 57600
MOTOR_IDS = [0, 3, 6, 9]
LEVEL_POS = "0 5 5 5"
CENTREZONE_POS = "3 8 8 8"
INNERZONE1_POS = "8 10 8 10"
INNERZONE2_POS = "8 13 8 8"
INNERZONE3_POS = "5 13 10 8"
INNERZONE4_POS = "3 13 13 8"
INNERZONE5_POS = "3 10 13 10"
INNERZONE6_POS = "3 8 13 13"
INNERZONE7_POS = "5 8 10 13"
INNERZONE8_POS = "8 8 8 13"
MIDDLEZONE1_POS = "18 14 8 14"
MIDDLEZONE2_POS = "18 23 8 8"
MIDDLEZONE3_POS = "6 23 14 8"
MIDDLEZONE4_POS = "3 23 23 8"
MIDDLEZONE5_POS = "3 14 23 14"
MIDDLEZONE6_POS = "3 8 23 23"
MIDDLEZONE7_POS = "9 8 14 23"
MIDDLEZONE8_POS = "18 8 8 23"
OUTERZONE1_POS = "18 16 8 16"
OUTERZONE2_POS = "18 23 8 8"
OUTERZONE3_POS = "11 23 16 8"
OUTERZONE4_POS = "3 23 23 8"
OUTERZONE5_POS = "3 16 23 16"
OUTERZONE6_POS = "3 8 23 23"
OUTERZONE7_POS = "11 8 16 23"
OUTERZONE8_POS = "18 8 8 23"

ADDR_PRESENT_POSITION = 36

# === Flags ===
bReact = True
bDebug = False
 
# === Initialize Port and Packet Handlers ===
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, 2)
 
if not portHandler.openPort():
    raise Exception("❌ Failed to open port")
 
if not portHandler.setBaudRate(BAUDRATE):
    raise Exception("❌ Failed to set baudrate")
 
print("✅ Port opened and baudrate set")
print("Controlling motors:", MOTOR_IDS)

ADDR_MX_MOVING_SPEED = 32
SERVO_SPEED = 1023
max_speed = 480

def set_moving_speed(motor_id: int, speed: int):
    dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
        portHandler,
        motor_id,
        ADDR_MX_MOVING_SPEED,
        speed
    )
    if dxl_comm != COMM_SUCCESS:
        print(f"❌ Motor {motor_id} speed set failed")
    elif dxl_err != 0:
        print(f"❌ Motor {motor_id} reported error")
    else:
        print(f"✅ Motor {motor_id} speed set to {speed}")

set_moving_speed(0, SERVO_SPEED)
set_moving_speed(3, SERVO_SPEED)
set_moving_speed(6, SERVO_SPEED)  
set_moving_speed(9, SERVO_SPEED)

def angle_to_dxl(motor_id, angle_deg):
    max_deg = 250
    resolution = 0.06
    angle_deg = max(0, min(max_deg, angle_deg))
    return int(angle_deg / resolution)

def motorMove(value_input):
    angle_vals = list(map(float, value_input.strip().split()))
    if len(angle_vals) != 4:
        print("⚠️ Please enter 4 angles.")
        
    groupSyncWrite.clearParam()
 
    for motor_id, angle in zip(MOTOR_IDS, angle_vals):
        dxl_position = angle_to_dxl(motor_id, angle)
        param_goal_position = [DXL_LOBYTE(dxl_position), DXL_HIBYTE(dxl_position)]
        success = groupSyncWrite.addParam(motor_id, param_goal_position)
        if not success:
            print(f"❌ Failed to add param for motor {motor_id}")
 
    groupSyncWrite.txPacket()

def main():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, 30)
    print("驱动返回 FPS =", cap.get(cv2.CAP_PROP_FPS))

    # === NEW: Background subtractor ===
    fgbg = cv2.createBackgroundSubtractorMOG2(
        history=500,
        varThreshold=50,
        detectShadows=False
    )

    start_time = time.time()

    r_target    = 57
    r_threshold = 26

    recorded_fall_point = False
    prev_center = prev_radius = prev_time = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧，退出程序")
            break

        # === NEW: Motion detection instead of HSV ===
        fgmask = fgbg.apply(frame)

        kernel = np.ones((5, 5), np.uint8)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.dilate(fgmask, kernel, iterations=2)

        mask = fgmask

        # Wait for background to stabilize
        if time.time() - start_time < 2:
            continue

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter noise
        contours = [c for c in contours if cv2.contourArea(c) > 500]

        if contours:
            (x, y), r = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
            r = int(r)
            if r > 3:
                center = (int(x), int(y))
                cv2.circle(frame, center, int(r), (0, 255, 255), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), -1)

                if prev_center is not None:
                    cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                    x_pred = cx + (x - cx) / r * r_target
                    y_pred = cy + (y - cy) / r * r_target
                    
                    deltaX = x_pred - 320
                    deltaY = y_pred - 240
                    pt_angle = math.atan2(deltaX,-deltaY)

                    if r > r_threshold:
                        if r < prev_radius:
                            user_input = LEVEL_POS
                            motorMove(user_input)

                        elif r > prev_radius:
                            recorded_fall_point = True

                            if (((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5) < 50:
                                if bReact:
                                    motorMove(CENTREZONE_POS)

                            elif (((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5) < 240:
                                if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                    if bReact: motorMove(INNERZONE1_POS)
                                elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
                                    if bReact: motorMove(INNERZONE2_POS)
                                elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
                                    if bReact: motorMove(INNERZONE3_POS)
                                elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
                                    if bReact: motorMove(INNERZONE4_POS)
                                elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
                                    if bReact: motorMove(INNERZONE5_POS)
                                elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
                                    if bReact: motorMove(INNERZONE6_POS)
                                elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
                                    if bReact: motorMove(INNERZONE7_POS)
                                else:
                                    if bReact: motorMove(INNERZONE8_POS)

                            elif (((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5) < 480:
                                if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                    if bReact: motorMove(MIDDLEZONE1_POS)
                                elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
                                    if bReact: motorMove(MIDDLEZONE2_POS)
                                elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
                                    if bReact: motorMove(MIDDLEZONE3_POS)
                                elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
                                    if bReact: motorMove(MIDDLEZONE4_POS)
                                elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
                                    if bReact: motorMove(MIDDLEZONE5_POS)
                                elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
                                    if bReact: motorMove(MIDDLEZONE6_POS)
                                elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
                                    if bReact: motorMove(MIDDLEZONE7_POS)
                                else:
                                    if bReact: motorMove(MIDDLEZONE8_POS)

                            else:
                                if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                    if bReact: motorMove(OUTERZONE1_POS)
                                elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
                                    if bReact: motorMove(OUTERZONE2_POS)
                                elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
                                    if bReact: motorMove(OUTERZONE3_POS)
                                elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
                                    if bReact: motorMove(OUTERZONE4_POS)
                                elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
                                    if bReact: motorMove(OUTERZONE5_POS)
                                elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
                                    if bReact: motorMove(OUTERZONE6_POS)
                                elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
                                    if bReact: motorMove(OUTERZONE7_POS)
                                else:
                                    if bReact: motorMove(OUTERZONE8_POS)

                prev_center, prev_radius, prev_time = center, r, time.time()
        
        else:
            motorMove(LEVEL_POS)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    motorMove(LEVEL_POS)

if __name__ == "__main__":
    main()