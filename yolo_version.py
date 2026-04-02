import cv2
import numpy as np
import time
import math
from ultralytics import YOLO

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

ADDR_MX_MOVING_SPEED = 32
SERVO_SPEED = 1023

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

# Set motor speeds
set_moving_speed(0, SERVO_SPEED)
set_moving_speed(3, SERVO_SPEED)
set_moving_speed(6, SERVO_SPEED)  
set_moving_speed(9, SERVO_SPEED)

# === Angle Conversion Function ===
def angle_to_dxl(motor_id, angle_deg):
    max_deg = 250
    resolution = 0.06
    angle_deg = max(0, min(max_deg, angle_deg))  # Clamp angle
    return int(angle_deg / resolution)

def motorMove(value_input):
    angle_vals = list(map(float, value_input.strip().split()))
    if len(angle_vals) != 4:
        print("⚠️ Please enter 4 angles.")
        return
    
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

def get_zone_from_position(x_pred, y_pred, pt_angle):
    """Determine which zone the ball will land in"""
    frame_center_x, frame_center_y = 320, 240
    distance_from_center = math.sqrt((x_pred - frame_center_x)**2 + (y_pred - frame_center_y)**2)
    
    if distance_from_center < 50:
        return CENTREZONE_POS
    elif distance_from_center < 240:
        return get_inner_zone(pt_angle)
    elif distance_from_center < 480:
        return get_middle_zone(pt_angle)
    else:
        return get_outer_zone(pt_angle)

def get_inner_zone(pt_angle):
    """Get inner zone based on angle"""
    if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
        return INNERZONE1_POS
    elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
        return INNERZONE2_POS
    elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
        return INNERZONE3_POS
    elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
        return INNERZONE4_POS
    elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
        return INNERZONE5_POS
    elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
        return INNERZONE6_POS
    elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
        return INNERZONE7_POS
    else:
        return INNERZONE8_POS

def get_middle_zone(pt_angle):
    """Get middle zone based on angle"""
    if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
        return MIDDLEZONE1_POS
    elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
        return MIDDLEZONE2_POS
    elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
        return MIDDLEZONE3_POS
    elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
        return MIDDLEZONE4_POS
    elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
        return MIDDLEZONE5_POS
    elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
        return MIDDLEZONE6_POS
    elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
        return MIDDLEZONE7_POS
    else:
        return MIDDLEZONE8_POS

def get_outer_zone(pt_angle):
    """Get outer zone based on angle"""
    if ((pt_angle < math.pi/8) and (pt_angle >= 0)) or ((pt_angle > -math.pi/8) and (pt_angle < 0)):
        return OUTERZONE1_POS
    elif ((pt_angle > -3*math.pi/8) and (pt_angle <= -math.pi/8)):
        return OUTERZONE2_POS
    elif ((pt_angle > -5*math.pi/8) and (pt_angle <= -3*math.pi/8)):
        return OUTERZONE3_POS
    elif ((pt_angle > -7*math.pi/8) and (pt_angle <= -5*math.pi/8)):
        return OUTERZONE4_POS
    elif ((pt_angle <= -7*math.pi/8) and (pt_angle > -math.pi)) or ((pt_angle >= 7*math.pi/8) and (pt_angle <= math.pi)):
        return OUTERZONE5_POS
    elif ((pt_angle >= 5*math.pi/8) and (pt_angle < 7*math.pi/8)):
        return OUTERZONE6_POS
    elif ((pt_angle >= 3*math.pi/8) and (pt_angle < 5*math.pi/8)):
        return OUTERZONE7_POS
    else:
        return OUTERZONE8_POS

def main():
    # Load YOLO model
    try:
        # Use a pre-trained model - you can train a custom model for better ping pong detection
        model = YOLO('yolov8n.pt')  # Using nano model for speed
        print("✅ YOLO model loaded successfully")
    except Exception as e:
        print(f"Failed to load YOLO model: {e}")
        print("Please install ultralytics: pip install ultralytics")
        return

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera")
        return
    
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, 30)
    print(f"Camera FPS = {cap.get(cv2.CAP_PROP_FPS):.1f}")

    r_target = 57         # Target radius for prediction
    r_threshold = 26      # Servo trigger threshold
    
    # COCO dataset class IDs
    SPORTS_BALL_CLASS = 32     
    ORANGE_CLASS = 48          
    APPLE_CLASS = 47           
    
    # Classes to detect as falling objects
    TARGET_CLASSES = [SPORTS_BALL_CLASS, ORANGE_CLASS, APPLE_CLASS]
    
    recorded_fall_point = False
    prev_center = prev_radius = prev_time = None
    detection_count = 0
    trigger_cooldown = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read video frame")
            break
        
        # Run YOLO detection
        results = model(frame, verbose=False)
        
        # Find target objects (ping pong balls, oranges, etc.)
        detected_objects = []
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    
                    # Check if it's one of our target classes
                    if class_id in TARGET_CLASSES and confidence > 0.4:
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        radius = int((x2 - x1) / 2)
                        
                        # Get class name
                        class_name = model.names[class_id]
                        
                        detected_objects.append({
                            'center': (center_x, center_y),
                            'radius': radius,
                            'confidence': confidence,
                            'class_id': class_id,
                            'class_name': class_name,
                            'bbox': (int(x1), int(y1), int(x2), int(y2))
                        })
        
        # Process the largest detected object (closest to camera)
        if detected_objects:
            # Sort by radius (larger = closer)
            best_object = max(detected_objects, key=lambda x: x['radius'])
            center = best_object['center']
            r = best_object['radius']
            class_name = best_object['class_name']
            confidence = best_object['confidence']
            
            # Draw detection
            x1, y1, x2, y2 = best_object['bbox']
            # Different colors for different objects
            if class_name == 'sports ball':
                color = (0, 165, 255)  # Orange for ping pong
                label = f'Ping Pong Ball'
            elif class_name == 'orange':
                color = (0, 100, 255)  # Dark orange for orange
                label = f'Orange'
            else:
                color = (0, 255, 255)  # Yellow for others
                label = f'{class_name}'
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.circle(frame, center, r, color, 2)
            cv2.putText(frame, f'{label}: {confidence:.2f}', 
                       (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            if bDebug:
                print(f"Detected: {label} at {center}, radius={r}, confidence={confidence:.2f}")
            
            # Calculate predicted landing point
            if prev_center is not None:
                cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                x_pred = cx + (center[0] - cx) / r * r_target
                y_pred = cy + (center[1] - cy) / r * r_target
                
                # Draw prediction point
                cv2.circle(frame, (int(x_pred), int(y_pred)), 5, (0, 255, 0), -1)
                cv2.line(frame, center, (int(x_pred), int(y_pred)), (0, 255, 0), 2)
                
                # Calculate the angle
                deltaX = x_pred - 320
                deltaY = y_pred - 240
                pt_angle = math.atan2(deltaX, -deltaY)
                
                # Trigger servo when object reaches threshold
                if r > r_threshold and trigger_cooldown == 0:
                    if r < prev_radius:
                        # Object moving away - level the platform
                        if bDebug:
                            print(f"Object moving: {center}, r={r:.1f} → Predicted: ({x_pred:.1f}, {y_pred:.1f})")
                        motorMove(LEVEL_POS)
                    
                    elif r > prev_radius:
                        # Object approaching - trigger servo
                        recorded_fall_point = True
                        if bDebug:
                            print(f"★★ TRIGGERING SERVO | {label} | r={r:.1f} ≥ {r_threshold} | Landing: ({x_pred:.1f},{y_pred:.1f}) ★★")
                        
                        # Determine zone and move servo
                        zone_position = get_zone_from_position(x_pred, y_pred, pt_angle)
                        if bReact:
                            motorMove(zone_position)
                            detection_count += 1
                            trigger_cooldown = 30
                            print(f"🎯 {label.upper()} DETECTED! Zone triggered. Count: {detection_count}")
                
                # Draw trigger threshold circle
                cv2.circle(frame, (cx, cy), r_threshold, (255, 0, 0), 2)
                cv2.putText(frame, f'Trigger: {r}/{r_threshold}', (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            # Update previous frame data
            prev_center, prev_radius, prev_time = center, r, time.time()
            
        else:
            # No object detected - level the platform
            if trigger_cooldown == 0:
                motorMove(LEVEL_POS)
        
        # Decrease cooldown
        if trigger_cooldown > 0:
            trigger_cooldown -= 1
        
        # Display statistics
        cv2.putText(frame, f'Falling Objects Detected: {detection_count}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'Cooldown: {trigger_cooldown}', (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show frame
        cv2.imshow("Falling Object Detection with YOLO", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    motorMove(LEVEL_POS)

if __name__ == "__main__":
    main()