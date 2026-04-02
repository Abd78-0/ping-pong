import cv2
import numpy as np
import time

def main():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return
    
    cap.set(cv2.CAP_PROP_FPS, 30)

    fgbg = cv2.createBackgroundSubtractorMOG2(
        history=500,
        varThreshold=50,
        detectShadows=False
    )

    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        fgmask = fgbg.apply(frame)

        kernel = np.ones((5, 5), np.uint8)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.dilate(fgmask, kernel, iterations=2)

        # Wait for background learning
        if time.time() - start_time < 2:
            cv2.putText(frame, "Learning background...", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", fgmask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = [c for c in contours if cv2.contourArea(c) > 200]

        for c in contours:
            area = cv2.contourArea(c)

            (x, y), r = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))

            # Draw circle
            cv2.circle(frame, center, int(r), (0, 255, 255), 2)
            cv2.circle(frame, center, 3, (0, 0, 255), -1)

            # Display values
            text = f"Area: {int(area)}  Radius: {int(r)}"
            cv2.putText(frame, text, (center[0]+10, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # Print to console (important!)
            print(f"Area = {int(area)}, Radius = {int(r)}")

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", fgmask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()