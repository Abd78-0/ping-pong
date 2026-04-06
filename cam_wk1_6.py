import cv2
from ultralytics import YOLO

def main():
    # Load YOLO model
    model = YOLO("yolo26n.pt")  # or your custom model
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Cannot open camera")
        return

    print("✅ Camera started. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        # Run YOLO detection
        results = model(frame)#, imgsz=320, conf=0.5, verbose=True)

        detected = False

        for res in results:
            if res.boxes is None:
                continue

            for box in res.boxes:
                cls = int(box.cls)

                # COCO class 32 = sports ball
                if cls not in [49, 32, 54, 47, 45]:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Compute center + pseudo radius
                w = x2 - x1
                h = y2 - y1
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                r = int(max(w, h) / 2)

                # Draw detection
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

                # Display info
                text = f"Ball | r={r}"
                cv2.putText(frame, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                detected = True
                break

            if detected:
                break

        # Show frame
        cv2.imshow("YOLO Ball Detection", frame)

        # Quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(res.names)
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()