import cv2
import numpy as np
import time
import math

from ultralytics import YOLO
from dynamixel_sdk import *

# === Constants ===
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
DEVICENAME = '/dev/ttyUSB0'
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
ADDR_MX_MOVING_SPEED = 32

bReact = True
bDebug = False

# === Dynamixel Setup ===
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, 2)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

def set_moving_speed(motor_id, speed):
    packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_MX_MOVING_SPEED, speed)

for mid in MOTOR_IDS:
    set_moving_speed(mid, 1023)

def angle_to_dxl(motor_id, angle_deg):
    max_deg = 250
    resolution = 0.06
    angle_deg = max(0, min(max_deg, angle_deg))
    return int(angle_deg / resolution)

def motorMove(value_input):
    angle_vals = list(map(float, value_input.strip().split()))
    groupSyncWrite.clearParam()

    for motor_id, angle in zip(MOTOR_IDS, angle_vals):
        dxl_position = angle_to_dxl(motor_id, angle)
        param_goal_position = [DXL_LOBYTE(dxl_position), DXL_HIBYTE(dxl_position)]
        groupSyncWrite.addParam(motor_id, param_goal_position)

    groupSyncWrite.txPacket()

# === MAIN ===
def main():
    cap = cv2.VideoCapture(0)

    model = YOLO("yolo26n.pt")  # replace with your custom model if needed

    r_target = 57
    r_threshold = 26

    prev_center = None
    prev_radius = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, imgsz=320, conf=0.5, verbose=False)

        detected = False

        for res in results:
            if res.boxes is None:
                continue

            for box in res.boxes:
                cls = int(box.cls[0])

                # COCO class 32 = sports ball
                if cls not in [49, 32, 54, 47, 45]:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                w = x2 - x1
                h = y2 - y1

                center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                r = int(max(w, h) / 2)

                # Optional smoothing
                if prev_radius is not None:
                    r = int(0.7 * prev_radius + 0.3 * r)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.circle(frame, center, 3, (0,0,255), -1)

                detected = True
                break
            if detected:
                break

        if detected and r > 3:
            if prev_center is not None:

                cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                x_pred = cx + (center[0] - cx) / r * r_target
                y_pred = cy + (center[1] - cy) / r * r_target

                deltaX = x_pred - 320
                deltaY = y_pred - 240
                pt_angle = math.atan2(deltaX, -deltaY)

                if r > r_threshold:
                    if r < prev_radius:
                        motorMove(LEVEL_POS)

                    else:
                        dist = ((x_pred - 320)**2 + (y_pred - 240)**2) ** 0.5

                        if dist < 50:
                            motorMove(CENTREZONE_POS)

                        elif dist < 240:
                            # INNER RING (same logic as yours)
                            if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                motorMove(INNERZONE1_POS)
                            elif (pt_angle > -3*math.pi/8):
                                motorMove(INNERZONE2_POS)
                            elif (pt_angle > -5*math.pi/8):
                                motorMove(INNERZONE3_POS)
                            elif (pt_angle > -7*math.pi/8):
                                motorMove(INNERZONE4_POS)
                            elif (pt_angle <= -7*math.pi/8 or pt_angle >= 7*math.pi/8):
                                motorMove(INNERZONE5_POS)
                            elif (pt_angle >= 5*math.pi/8):
                                motorMove(INNERZONE6_POS)
                            elif (pt_angle >= 3*math.pi/8):
                                motorMove(INNERZONE7_POS)
                            else:
                                motorMove(INNERZONE8_POS)

                        elif dist < 480:
                            # MIDDLE RING
                            if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                motorMove(MIDDLEZONE1_POS)
                            elif (pt_angle > -3*math.pi/8):
                                motorMove(MIDDLEZONE2_POS)
                            elif (pt_angle > -5*math.pi/8):
                                motorMove(MIDDLEZONE3_POS)
                            elif (pt_angle > -7*math.pi/8):
                                motorMove(MIDDLEZONE4_POS)
                            elif (pt_angle <= -7*math.pi/8 or pt_angle >= 7*math.pi/8):
                                motorMove(MIDDLEZONE5_POS)
                            elif (pt_angle >= 5*math.pi/8):
                                motorMove(MIDDLEZONE6_POS)
                            elif (pt_angle >= 3*math.pi/8):
                                motorMove(MIDDLEZONE7_POS)
                            else:
                                motorMove(MIDDLEZONE8_POS)

                        else:
                            # OUTER RING
                            if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
                                motorMove(OUTERZONE1_POS)
                            elif (pt_angle > -3*math.pi/8):
                                motorMove(OUTERZONE2_POS)
                            elif (pt_angle > -5*math.pi/8):
                                motorMove(OUTERZONE3_POS)
                            elif (pt_angle > -7*math.pi/8):
                                motorMove(OUTERZONE4_POS)
                            elif (pt_angle <= -7*math.pi/8 or pt_angle >= 7*math.pi/8):
                                motorMove(OUTERZONE5_POS)
                            elif (pt_angle >= 5*math.pi/8):
                                motorMove(OUTERZONE6_POS)
                            elif (pt_angle >= 3*math.pi/8):
                                motorMove(OUTERZONE7_POS)
                            else:
                                motorMove(OUTERZONE8_POS)

            prev_center = center
            prev_radius = r

        else:
            motorMove(LEVEL_POS)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    motorMove(LEVEL_POS)

if __name__ == "__main__":
    main()