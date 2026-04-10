import cv2
import numpy as np
import time

def main():
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
        if not ret:
            print("无法读取视频帧，退出程序")
            break

        # 将 BGR 转换为 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 颜色阈值：根据你的乒乓球颜色微调（此处示例橙/黄球）
        lower_color = np.array([0, 74, 215])   # HSV下限
        upper_color = np.array([18, 255, 255])  # HSV上限

        # 生成二值掩膜
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # 形态学操作：腐蚀、膨胀去除噪点
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 找到面积最大的轮廓，假设它就是球
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius_pixels = cv2.minEnclosingCircle(largest_contour)

            if radius_pixels > 3:
                center = (int(x), int(y))
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
        cv2.imshow("Mask", mask)  # 调试用

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
