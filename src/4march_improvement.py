import cv2
import numpy as np
import time
import math
from collections import deque

def main():
    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Failed to open camera")
        return
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print(f"📷 Camera FPS = {cap.get(cv2.CAP_PROP_FPS):.1f}")
    print(f"📷 Camera Resolution = {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    
    # Create background subtractor
    backSub = cv2.createBackgroundSubtractorMOG2(
        history=500,        # Number of frames to learn background
        varThreshold=36,    # Threshold for shadow detection
        detectShadows=True  # Detect shadows
    )
    
    # Parameters for falling object detection
    r_target = 57          # Target radius for prediction
    r_threshold = 26       # Trigger threshold
    min_object_area = 100   # Minimum area for small objects
    max_object_area = 2000  # Maximum area for small objects
    
    # Size change tracking parameters
    size_history = deque(maxlen=10)  # Store last 10 sizes
    position_history = deque(maxlen=10)  # Store last 10 positions
    size_growth_threshold = 1.2  # 20% growth indicates falling towards camera
    min_growth_frames = 3  # Need at least 3 frames of growth to confirm
    
    # Variables
    bDebug = True
    show_fps = True
    show_foreground = True
    frame_count = 0
    fps_start_time = time.time()
    fps = 0
    
    # For tracking objects
    tracked_objects = {}  # object_id: {'centers': deque, 'sizes': deque, 'first_seen': time}
    next_object_id = 0
    falling_objects = []  # Currently falling objects
    detection_count = 0
    trigger_cooldown = 0
    last_trigger_time = 0
    
    print("\n🎯 Falling Object Detection - Background Subtraction Mode")
    print("Detects small objects that grow in size (falling towards camera)")
    print("\nThe system will learn the background for 2 seconds")
    print("After learning, any small moving object that grows in size will be detected")
    print("\nControls:")
    print("  - Press 'q' to quit")
    print("  - Press 'd' to toggle debug mode")
    print("  - Press 'f' to toggle foreground mask window")
    print("  - Press 'r' to reset background model")
    print("  - Press 'h' to show/hide help")
    print("\nLearning background... Please keep camera still for 2 seconds\n")
    
    help_visible = True
    
    # Warm up the background subtractor
    warmup_frames = 60  # About 2 seconds at 30fps
    for i in range(warmup_frames):
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to read camera during warmup")
            return
        backSub.apply(frame)
        if i % 10 == 0:
            print(f"Learning background: {i}/{warmup_frames} frames")
    
    print("✅ Background learning complete! Now detecting falling objects...\n")
    
    def calculate_growth_rate(sizes):
        """Calculate if object is growing in size"""
        if len(sizes) < min_growth_frames:
            return False, 0
        
        # Get recent sizes (last 3-5 frames)
        recent_sizes = list(sizes)[-min_growth_frames:]
        
        # Check if size is consistently increasing
        is_growing = True
        growth_rate = 1.0
        
        for i in range(1, len(recent_sizes)):
            if recent_sizes[i] <= recent_sizes[i-1]:
                is_growing = False
                break
        
        if is_growing and len(recent_sizes) >= 2:
            # Calculate growth rate (current size / initial size)
            growth_rate = recent_sizes[-1] / recent_sizes[0]
            is_growing = growth_rate > size_growth_threshold
        
        return is_growing, growth_rate
    
    def calculate_vertical_motion(positions):
        """Check if object is moving downward (falling)"""
        if len(positions) < 3:
            return False
        
        # Check if y-coordinate is increasing (moving down in frame)
        recent_y = [p[1] for p in list(positions)[-3:]]
        
        # Check if y is consistently increasing
        is_falling = all(recent_y[i] > recent_y[i-1] for i in range(1, len(recent_y)))
        
        return is_falling
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to read video frame")
            break
        
        frame_count += 1
        
        # Calculate FPS
        if frame_count % 30 == 0:
            current_time = time.time()
            fps = 30 / (current_time - fps_start_time)
            fps_start_time = current_time
        
        # Apply background subtraction
        fgmask = backSub.apply(frame)
        
        # Clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        fgmask = cv2.erode(fgmask, kernel, iterations=1)  # Remove noise
        fgmask = cv2.dilate(fgmask, kernel, iterations=2)  # Fill gaps
        
        # Find contours of moving objects
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Current frame objects
        current_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area (small objects only)
            if min_object_area < area < max_object_area:
                # Get the minimum enclosing circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                radius = int(radius)
                center = (int(x), int(y))
                
                # Calculate shape properties
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                else:
                    circularity = 0
                
                current_objects.append({
                    'center': center,
                    'radius': radius,
                    'area': area,
                    'circularity': circularity,
                    'contour': contour
                })
        
        # Match current objects with tracked objects
        matched_ids = set()
        
        for obj in current_objects:
            center = obj['center']
            matched = False
            
            # Try to match with existing tracked objects
            for obj_id, tracked in tracked_objects.items():
                if obj_id in matched_ids:
                    continue
                
                last_center = tracked['centers'][-1] if tracked['centers'] else None
                if last_center:
                    # Calculate distance between centers
                    distance = math.sqrt((center[0] - last_center[0])**2 + (center[1] - last_center[1])**2)
                    
                    # If within reasonable distance, match
                    if distance < 50:
                        # Update tracked object
                        tracked['centers'].append(center)
                        tracked['sizes'].append(obj['area'])
                        tracked['last_seen'] = time.time()
                        tracked['circularity'] = obj['circularity']
                        tracked['radius'] = obj['radius']
                        
                        matched_ids.add(obj_id)
                        matched = True
                        break
            
            # If not matched, create new tracked object
            if not matched:
                tracked_objects[next_object_id] = {
                    'centers': deque([center], maxlen=10),
                    'sizes': deque([obj['area']], maxlen=10),
                    'first_seen': time.time(),
                    'last_seen': time.time(),
                    'circularity': obj['circularity'],
                    'radius': obj['radius']
                }
                next_object_id += 1
        
        # Remove old objects (not seen for 1 second)
        current_time = time.time()
        tracked_objects = {obj_id: obj for obj_id, obj in tracked_objects.items() 
                          if current_time - obj['last_seen'] < 1.0}
        
        # Detect falling objects
        falling_objects = []
        for obj_id, obj in tracked_objects.items():
            # Check if object is growing (falling towards camera)
            is_growing, growth_rate = calculate_growth_rate(obj['sizes'])
            
            # Check if object is moving downward
            is_falling = calculate_vertical_motion(obj['centers'])
            
            # Object is considered falling if it's growing OR moving downward
            is_falling_object = is_growing or is_falling
            
            if is_falling_object:
                falling_objects.append({
                    'id': obj_id,
                    'center': obj['centers'][-1],
                    'radius': obj['radius'],
                    'area': obj['sizes'][-1],
                    'growth_rate': growth_rate,
                    'is_growing': is_growing,
                    'is_falling': is_falling,
                    'circularity': obj['circularity']
                })
        
        # Process falling objects
        if falling_objects:
            # Sort by area to get the largest (closest) object
            best_object = max(falling_objects, key=lambda x: x['area'])
            center = best_object['center']
            r = best_object['radius']
            growth_rate = best_object['growth_rate']
            is_growing = best_object['is_growing']
            is_falling = best_object['is_falling']
            circularity = best_object['circularity']
            
            # Draw detection with different colors based on state
            if is_growing and is_falling:
                color = (0, 0, 255)  # Red - actively falling and growing
                status = "FALLING & GROWING"
            elif is_growing:
                color = (0, 165, 255)  # Orange - growing (falling towards camera)
                status = "FALLING TOWARDS CAMERA"
            elif is_falling:
                color = (0, 255, 255)  # Yellow - moving downward
                status = "MOVING DOWN"
            else:
                color = (255, 255, 0)  # Light blue - other motion
                status = "MOVING"
            
            cv2.circle(frame, center, r, color, 2)
            cv2.circle(frame, center, 3, (0, 0, 255), -1)
            cv2.putText(frame, f'{status}', (center[0]-50, center[1]-15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            if bDebug:
                print(f"🔍 Object {best_object['id']}: center={center}, radius={r}, area={best_object['area']:.0f}")
                print(f"   Growth rate: {growth_rate:.2f}, Growing: {is_growing}, Falling: {is_falling}")
            
            # Calculate predicted landing point
            if len(tracked_objects[best_object['id']]['centers']) >= 2:
                centers = list(tracked_objects[best_object['id']]['centers'])
                prev_center = centers[-2] if len(centers) >= 2 else None
                
                if prev_center is not None:
                    cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                    
                    # Predict where the object will land
                    x_pred = cx + (center[0] - cx) / r * r_target
                    y_pred = cy + (center[1] - cy) / r * r_target
                    
                    # Draw prediction
                    cv2.circle(frame, (int(x_pred), int(y_pred)), 5, (0, 255, 0), -1)
                    cv2.line(frame, center, (int(x_pred), int(y_pred)), (0, 255, 0), 2)
                    
                    # Calculate the angle
                    deltaX = x_pred - cx
                    deltaY = y_pred - cy
                    pt_angle = math.atan2(deltaX, -deltaY)
                    angle_deg = pt_angle * 180 / math.pi
                    
                    # Determine zone
                    distance_from_center = math.sqrt((x_pred - cx)**2 + (y_pred - cy)**2)
                    if distance_from_center < 50:
                        zone = "CENTER"
                    elif distance_from_center < 240:
                        zone = "INNER RING"
                    elif distance_from_center < 480:
                        zone = "MIDDLE RING"
                    else:
                        zone = "OUTER RING"
                    
                    # Display zone and angle
                    cv2.putText(frame, f'Zone: {zone}', (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(frame, f'Angle: {angle_deg:.1f}°', (10, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    cv2.putText(frame, f'Growth: {growth_rate:.2f}x', (10, 150), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
                    
                    # Trigger servo when object is growing AND reaches threshold
                    if is_growing and r > r_threshold and trigger_cooldown == 0:
                        cv2.putText(frame, 'TRIGGER!', (center[0]-40, center[1]-30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        cv2.putText(frame, f'Servo Moving to {zone}', (10, 180), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        
                        if bDebug:
                            print(f"⭐ TRIGGER! Falling object detected")
                            print(f"   Zone: {zone}, Angle: {angle_deg:.1f}°, Growth: {growth_rate:.2f}x")
                        
                        detection_count += 1
                        trigger_cooldown = 30  # Cooldown to prevent multiple triggers
                    
                    # Draw trigger threshold circle
                    cv2.circle(frame, (cx, cy), r_threshold, (255, 0, 0), 2)
                    cv2.putText(frame, f'Size: {r}/{r_threshold}', (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                    
                    # Draw angle indicator
                    angle_end_x = int(cx + 100 * math.cos(pt_angle))
                    angle_end_y = int(cy - 100 * math.sin(pt_angle))
                    cv2.line(frame, (cx, cy), (angle_end_x, angle_end_y), (255, 255, 0), 2)
        
        else:
            # No falling objects detected
            if bDebug and frame_count % 30 == 0:
                print("👀 No falling objects detected")
        
        # Decrease cooldown
        if trigger_cooldown > 0:
            trigger_cooldown -= 1
            cv2.putText(frame, f'Cooldown: {trigger_cooldown}', (10, 210), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Display statistics
        if show_fps:
            cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.putText(frame, f'Falling Objects: {detection_count}', (10, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f'Tracked Objects: {len(tracked_objects)}', (10, 270), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show foreground mask
        if show_foreground:
            fgmask_colored = cv2.cvtColor(fgmask, cv2.COLOR_GRAY2BGR)
            cv2.putText(fgmask_colored, 'Moving Objects Mask', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(fgmask_colored, f'Falling: growth > {size_growth_threshold}x', (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            cv2.imshow("Foreground Mask", fgmask_colored)
        
        # Show help text
        if help_visible:
            help_y = frame.shape[0] - 100
            cv2.rectangle(frame, (5, help_y-5), (frame.shape[1]-5, frame.shape[0]-5), (0, 0, 0), -1)
            cv2.putText(frame, 'Controls:', (10, help_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, 'q-Quit | d-Debug | f-Foreground | r-Reset BG | h-Help', 
                       (10, help_y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Show main window
        cv2.imshow("Falling Object Detection", frame)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("\n👋 Exiting...")
            break
        elif key == ord('d'):
            bDebug = not bDebug
            print(f"🐛 Debug mode: {'ON' if bDebug else 'OFF'}")
        elif key == ord('f'):
            show_foreground = not show_foreground
            if not show_foreground:
                cv2.destroyWindow("Foreground Mask")
            print(f"🎨 Foreground mask window: {'ON' if show_foreground else 'OFF'}")
        elif key == ord('r'):
            # Reset background model
            backSub = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=36, detectShadows=True)
            tracked_objects.clear()
            print("🔄 Background model reset - learning new background...")
            for i in range(30):
                ret, frame = cap.read()
                if ret:
                    backSub.apply(frame)
            print("✅ Background learning complete!")
        elif key == ord('h'):
            help_visible = not help_visible
            print(f"📖 Help text: {'visible' if help_visible else 'hidden'}")
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print(f"\n📊 Test Summary:")
    print(f"   - Falling objects detected: {detection_count}")
    print(f"   - Final FPS: {fps:.1f}")
    print("✅ Testing completed")

if __name__ == "__main__":
    main()
