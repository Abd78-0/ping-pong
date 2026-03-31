import cv2
import numpy as np
import time
import math

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头")

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FPS, 60)
print("驱动返回 FPS =", cap.get(cv2.CAP_PROP_FPS))

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取视频帧，退出程序")
        break
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 70, 217]), np.array([30, 255, 255]))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        (x, y), r = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
        r = int(r)
        r_target    = 57         # 触板最大半径
        if r > 3:
            center = (int(x), int(y))
            cv2.circle(frame, center, int(r), (0, 255, 255), 2)
            cv2.circle(frame, center, 2, (0, 0, 255), -1)
            cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
            x_pred = cx + (x - cx) / r * r_target
            y_pred = cy + (y - cy) / r * r_target
            print(f"Centre of the ball:({center[0]}, {center[1]}), r={r:.1f} → Predicted falling point:({x_pred:.1f}, {y_pred:.1f})")
            
    
    cv2.imshow("Camera Upward View", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
cap.release()
cv2.destroyAllWindows()    
