import cv2
import numpy as np
import time
from ultralytics import YOLO


model = YOLO("yolov8n.pt")
font = cv2.FONT_HERSHEY_SIMPLEX
label = "orange"

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    frame_count = 0

    detect1 = 0
    detect2 = 0

    prev_center = None
    prev_radius = None
    prev_time = None

    target_radius = 360

    predicting = True
    start = time.time()
    while True:
        ret, frame = cap.read()
    
        frame_count += 1
        if not ret:
            print("No frame detetced")
            break

        frame = cv2.flip(frame,1)
        results = model(frame, imgsz=320,conf=0.15, verbose=False)

        r = results[0]
        for i in range(len(r.boxes)):
            cls_idx = int(r.boxes.cls[i])
            name = r.names[cls_idx]

            if name == label:
                x1,y1,x2,y2 = map(int,r.boxes.xyxy[i])
                x, y, radius_pixels = x1,y1, (((x1-x2) + (y1-y2))//4)*(-1)

                if radius_pixels > 3:
                    detect2 += 1
                    center = tuple(map(int,((x1+x2)//2, (y1+y2)//2)))
                    radius_int = int(radius_pixels)

                    cv2.circle(frame, center, radius_int, (0, 255, 255), 2)
                    cv2.circle(frame, center, 2, (0, 0, 255), -1)  

        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 74, 215]) 
        upper_color = np.array([18, 255, 255])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius_pixels = cv2.minEnclosingCircle(largest_contour)

            if radius_pixels > 3:
                center = (int(x), int(y))
                radius_int = int(radius_pixels)
                detect1 += 1
                cv2.circle(frame, center, radius_int, (0, 255, 255), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), -1)


                  
                if prev_radius is not None and prev_center is not None and prev_time is not None:
                    radius_change = radius_pixels - prev_radius
                    time_diff = time.time() - prev_time
                    

        cv2.imshow("Camera Upward View", frame)
        
        if cv2.waitKey(1) == ord('q'):
                end = time.time()
                time_taken = end - start
                print(f"Time taken is {time_taken:.2f} seconds")
                print(f"The FPS of the code is {(frame_count/time_taken):.2f}")
                print(f"HSV detected {detect1} times")
                print(f"YOLO detected {detect2} times")
                break
            
            

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

