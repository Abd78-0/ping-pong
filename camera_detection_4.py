import cv2
import numpy as np
import time
import math

def main():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, 30)
    print("驱动返回 FPS =", cap.get(cv2.CAP_PROP_FPS))

    # Background subtractor
    fgbg = cv2.createBackgroundSubtractorMOG2(
        history=500,
        varThreshold=50,
        detectShadows=False
    )

    start_time = time.time()

    r_target    = 57
    r_threshold = 26

    prev_center = None
    prev_radius = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧，退出程序")
            break

        # Motion detection
        fgmask = fgbg.apply(frame)

        kernel = np.ones((5, 5), np.uint8)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.dilate(fgmask, kernel, iterations=2)

        mask = fgmask

        # Wait for background to stabilize
        if time.time() - start_time < 2:
            cv2.putText(frame, "Learning background...", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter noise
        contours = [c for c in contours if cv2.contourArea(c) > 500]

        if contours:
            (x, y), r = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
            r = int(r)

            if r > 3:
                center = (int(x), int(y))

                # Draw detection
                cv2.circle(frame, center, r, (0, 255, 255), 2)
                cv2.circle(frame, center, 3, (0, 0, 255), -1)

                # Prediction logic (same as your code)
                if prev_center is not None:
                    cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                    x_pred = cx + (x - cx) / r * r_target
                    y_pred = cy + (y - cy) / r * r_target

                    # Draw predicted point
                    cv2.circle(frame, (int(x_pred), int(y_pred)), 5, (255, 0, 0), -1)

                    deltaX = x_pred - 320
                    deltaY = y_pred - 240
                    pt_angle = math.atan2(deltaX, -deltaY)

                    # Debug info
                    cv2.putText(frame, f"r={r}", (10,60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.putText(frame, f"angle={pt_angle:.2f}", (10,90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                prev_center = center
                prev_radius = r

        else:
            prev_center = None
            prev_radius = None

        # Show windows
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()