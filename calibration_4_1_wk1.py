import cv2
import numpy as np

def main():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    cap.set(cv2.CAP_PROP_FPS, 30)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # === CHANGE THIS RANGE BASED ON YOUR BALL COLOR ===
        lower = np.array([0, 70, 150])
        upper = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = [c for c in contours if cv2.contourArea(c) > 200]

        for c in contours:
            area = cv2.contourArea(c)
            (x, y), r = cv2.minEnclosingCircle(c)

            center = (int(x), int(y))

            # Draw
            cv2.circle(frame, center, int(r), (0, 255, 255), 2)
            cv2.circle(frame, center, 3, (0, 0, 255), -1)

            # Display values
            text = f"Area: {int(area)}  Radius: {int(r)}"
            cv2.putText(frame, text, (center[0]+10, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # Print to console
            print(f"Area = {int(area)}, Radius = {int(r)}")

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
