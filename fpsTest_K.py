import cv2
import numpy as np
import time
from ultralytics import YOLO
import time
 
model = YOLO("yolov8n.pt")
font = cv2.FONT_HERSHEY_SIMPLEX
label = "orange"
start = time.time()
frame_count = 0
 
def main(frame_count):
    # 打开前置摄像头（如果 index=0 不行，可尝试 1）
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
 
    # 记录上一帧的球心位置、半径和时间
    prev_center = None
    prev_radius = None
    prev_time = None
 
    # 设置目标半径（表示球触碰亚克力板时的半径，360像素）
    target_radius = 360
 
    # 标志变量，判断是否处于预测状态
    predicting = True
 
    while True:
        ret, frame = cap.read()
 
        frame_count += 1
        if not ret:
            print("无法读取视频帧，退出程序")
            break
 
        results = model(frame)
        r = results[0]
        for i in range(len(r.boxes)):
            cls_idx = int(r.boxes.cls[i])
            name = r.names[cls_idx]
 
            if name == label:
                x1,y1,x2,y2 = map(int,r.boxes.xyxy[i])
                x, y, radius_pixels = x1,y1, (((x1-x2) + (y1-y2))//4)*(-1)
 
                if radius_pixels > 3:
                    center = tuple(map(int,((x1+x2)//2, (y1+y2)//2)))
                    radius_int = int(radius_pixels)
 
                    # 在原图上画出圆
                    cv2.circle(frame, center, radius_int, (0, 255, 255), 2)
                    cv2.circle(frame, center, 2, (0, 0, 255), -1)
 
                    # === 始终实时输出球心坐标和半径 ===
                    print(f"Coordinate of Centre: ({center[0]}, {center[1]}), Radius: {radius_pixels:.1f}pixels")
 
                    # === 核心：根据像素半径的变化，估算球的落点 ===
                    if prev_radius is not None and prev_center is not None and prev_time is not None:
                        # 计算半径变化率（假设变化为线性）
                        radius_change = radius_pixels - prev_radius
                        time_diff = time.time() - prev_time
                       
                         
            # 显示结果
        cv2.imshow("Camera Upward View", frame)
       
        if cv2.waitKey(1) == ord('q'):
                return frame_count
           
           
 
    cap.release()
    cv2.destroyAllWindows()
 
if __name__ == "__main__":
    frame_count = main(0,start)
    end = time.time()
 
 
time_taken = end - start
print(f"\nThe total time the programme ran for, was {time_taken:.2f}seconds.")
print(f"Number of frames ran, is {frame_count}.")
print(f"The FPS of the the entire process is {(frame_count/time_taken):.2f}")
 
 
