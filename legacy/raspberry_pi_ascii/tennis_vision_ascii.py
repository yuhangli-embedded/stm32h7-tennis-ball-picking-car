from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
import imutils
from threading import Thread
from queue import Queue, Full
import os

# === 配置参数 ===
SERIAL_PORT  = '/dev/serial0'
BAUD_RATE    = 115200
EST_BALL_RADIUS = 43                # 实测50cm时像素半径
APPROACH_RADIUS = 43                # 判定进入接近阶段的r
TARGET_RADIUS   = 80                # 判定接近20cm时的r，可后期实测
BASE_CENTER_THRESHOLD = 20          # 基础中心位置阈值(像素)
CIRC_THRESH  = 0.5                  # 圆形度阈值
MIN_RAD      = int(EST_BALL_RADIUS * 0.5)
MAX_RAD      = int(EST_BALL_RADIUS * 1.5)
RES_WIDTH    = 640                  # 图像宽度
FRAME_HEIGHT = 480                  # 图像高度
FRAME_CENTER = (RES_WIDTH // 2, FRAME_HEIGHT // 2)

# 垂直偏移量补偿 - 摄像头倒置修正（正值表示将检测中心下移）
VERTICAL_OFFSET = 34

# 目标矩形框缩放比例 (默认1.5倍)
TARGET_BOX_SCALE = 1.5

# 定位成功条件
SUCCESS_RADIUS = (EST_BALL_RADIUS - 2, EST_BALL_RADIUS + 2)  # 半径范围 (41-45)
SUCCESS_DISTANCE = 5  # dx, dy的最大允许偏差

# HSV 阈值 (网球颜色范围)
LH, LS, LV = 67, 81, 69
UH, US, UV = 88, 255, 255

# 串口消息队列
QUEUE_MAX    = 10
MIN_DT_LOST  = 0.5     # 无球时帧率降低
MIN_DT_FOUND = 0.1     # 识别到球时提高速率

# 记录程序启动时间
start_time = time.time()
tx_queue = Queue(maxsize=QUEUE_MAX)
last_send_time = 0.0

# 状态跟踪
last_approach_flag = False
last_presence = False

def open_serial():
    try:
        if not os.path.exists(SERIAL_PORT):
            print(f"[ERROR] Serial device {SERIAL_PORT} does not exist!")
            return None
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5
        )
        time.sleep(2)
        if not ser.is_open:
            print(f"[ERROR] Serial port {SERIAL_PORT} failed to open")
            return None
        print(f"[INFO] Opened serial {SERIAL_PORT} @ {BAUD_RATE}bps")
        ser.write(b"<SYSTEM,START>\n")
        return ser
    except Exception as e:
        print(f"[ERROR] Cannot open serial port: {e}")
        return None

def serial_worker(ser):
    while True:
        try:
            msg = tx_queue.get(timeout=0.5)
            if msg is None:
                break
            retry = 0
            while retry < 3:
                try:
                    ser.write(msg)
                    print(f"[SERIAL] Sent: {msg.strip().decode()}")
                    break
                except Exception as e:
                    print(f"[SERIAL ERROR] Attempt {retry+1}: {e}")
                    time.sleep(0.05)
                    retry += 1
        except:
            pass
    if ser and ser.is_open:
        try:
            ser.close()
        except:
            pass
    print("[SERIAL] Worker thread exited")

def pack_frame_message(presence, dx, dy, radius, approach):
    return (
        f"<FRAME,t={int((time.time()-start_time)*1000)},"
        f"pres={int(presence)},dx={dx:.1f},dy={dy:.1f},"
        f"r={radius:.1f},app={int(approach)}>\n"
    ).encode('utf-8')

def pack_system_message(message):
    return f"<SYSTEM,{message}>\n".encode('utf-8')

def cleanup(picam2, ser, t):
    print("[CLEANUP] Shutting down...")
    cv2.destroyAllWindows()
    if picam2:
        try: picam2.stop()
        except: pass
    if ser and ser.is_open:
        try: 
            ser.write(b"<SYSTEM,EXIT>\n")
            time.sleep(0.1)  # 确保消息发送
        except: pass
        try: ser.close()
        except: pass
    if t and t.is_alive():
        try:
            tx_queue.put(None)
            t.join(timeout=2.0)
        except: pass
    print("[CLEANUP] Completed")

def main():
    global last_send_time, last_approach_flag, last_presence, TARGET_BOX_SCALE
    
    # 初始化摄像头
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (RES_WIDTH, FRAME_HEIGHT), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # 等待摄像头稳定

    # 打开串口
    ser = open_serial()
    if ser is None:
        print("[ERROR] Failed to open serial port. Exiting.")
        return
    
    # 启动串口工作线程
    t = Thread(target=serial_worker, args=(ser,), daemon=True)
    t.start()

    # 性能监控变量
    frame_count, fps_start, fps_text = 0, time.time(), ''
    MIN_DT = MIN_DT_LOST

    # 创建参数调整窗口
    cv2.namedWindow("Parameters")
    cv2.createTrackbar("LH", "Parameters", LH, 255, lambda x: None)
    cv2.createTrackbar("LS", "Parameters", LS, 255, lambda x: None)
    cv2.createTrackbar("LV", "Parameters", LV, 255, lambda x: None)
    cv2.createTrackbar("UH", "Parameters", UH, 255, lambda x: None)
    cv2.createTrackbar("US", "Parameters", US, 255, lambda x: None)
    cv2.createTrackbar("UV", "Parameters", UV, 255, lambda x: None)
    cv2.createTrackbar("OffsetY", "Parameters", VERTICAL_OFFSET + 50, 100, lambda x: None)
    # 添加目标框大小滑块 (范围1.0-3.0，步长0.1)
    cv2.createTrackbar("Box Scale", "Parameters", int(TARGET_BOX_SCALE * 10), 30, lambda x: None)
    
    # 计算调整后的中心点（补偿垂直偏移）
    adjusted_center = (FRAME_CENTER[0], FRAME_CENTER[1] + VERTICAL_OFFSET)
    
    # 定位成功计数器（避免瞬时满足条件）
    success_count = 0
    SUCCESS_THRESHOLD = 5  # 连续5帧满足条件才发送成功消息
    
    # 状态标志
    position_success_sent = False
    laser_guided_mode = False

    print(f"[INFO] Using vertical offset: {VERTICAL_OFFSET}px")
    print(f"[INFO] Success conditions: r in {SUCCESS_RADIUS}, |dx|,|dy| < {SUCCESS_DISTANCE}")
    print("[INFO] Starting detection loop. Press 'q' to quit.")
    try:
        while True:
            # 读取滑动条值
            lh = cv2.getTrackbarPos("LH", "Parameters")
            ls = cv2.getTrackbarPos("LS", "Parameters")
            lv = cv2.getTrackbarPos("LV", "Parameters")
            uh = cv2.getTrackbarPos("UH", "Parameters")
            us = cv2.getTrackbarPos("US", "Parameters")
            uv = cv2.getTrackbarPos("UV", "Parameters")
            offset_y = cv2.getTrackbarPos("OffsetY", "Parameters") - 50
            
            # 更新目标框大小
            box_scale_val = cv2.getTrackbarPos("Box Scale", "Parameters")
            TARGET_BOX_SCALE = box_scale_val / 10.0
            
            # 更新垂直偏移
            adjusted_center = (FRAME_CENTER[0], FRAME_CENTER[1] + offset_y)
            
            # === 关键修改 ===
            # 动态调整中心区域阈值：基础阈值 * 目标框缩放比例
            center_threshold = int(BASE_CENTER_THRESHOLD * TARGET_BOX_SCALE)
            
            # 捕获图像帧
            frame = picam2.capture_array()
            if frame is None:
                continue
            
            # 调整大小
            frame = imutils.resize(frame, width=RES_WIDTH)
            
            # 预处理
            blurred = cv2.medianBlur(frame, 5)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # 颜色阈值分割 (使用滑动条值)
            mask = cv2.inRange(hsv, (lh, ls, lv), (uh, us, uv))
            
            # 形态学操作
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=1)
            
            # 查找轮廓
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            
            # 当前时间
            now = time.time()
            
            # 初始化变量
            presence, dx, dy, r = False, 0.0, 0.0, 0.0
            approach_flag = False
            stage_text = "Searching"
            best_contour = None
            best_circ = 0.0

            # 处理轮廓
            if cnts:
                for c in cnts:
                    area = cv2.contourArea(c)
                    if area < 100:  # 忽略小面积噪声
                        continue
                    
                    peri = cv2.arcLength(c, True)
                    circ = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
                    
                    # 最小外接圆
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    
                    # 筛选条件：半径范围和圆形度
                    if MIN_RAD <= radius <= MAX_RAD and circ > CIRC_THRESH:
                        if circ > best_circ:
                            best_circ = circ
                            best_contour = c
                            cx, cy = x, y
                            presence = True
            
            # 处理最佳轮廓
            if best_contour is not None:
                (x, y), radius = cv2.minEnclosingCircle(best_contour)
                cx, cy = x, y
                
                # 摄像头倒置修正：坐标计算取反
                dx = adjusted_center[0] - cx  # X方向取反
                dy = adjusted_center[1] - cy  # Y方向取反
                
                r = radius
                presence = True

                # === 关键修改 ===
                # 使用动态调整的中心阈值判断状态
                if abs(dx) < center_threshold and abs(dy) < center_threshold:
                    if r >= TARGET_RADIUS:
                        approach_flag = False
                        stage_text = "STOP (20cm)"
                    elif r >= APPROACH_RADIUS:
                        approach_flag = True
                        stage_text = "APPROACH (to 20cm)"
                    else:
                        approach_flag = True
                        stage_text = "ALIGN (50~150cm)"
                else:
                    approach_flag = False
                    stage_text = "CENTERING"

                # 绘制轮廓和中心点
                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                cv2.putText(frame, stage_text, (int(cx) + 20, int(cy)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                # 添加方向指示箭头（摄像头倒置修正版）
                # 使用动态调整的阈值作为箭头触发条件
                arrow_threshold = center_threshold * 2  # 箭头在2倍中心阈值时显示
                if abs(dx) > arrow_threshold or abs(dy) > arrow_threshold:
                    if dx > arrow_threshold:
                        cv2.arrowedLine(frame, (adjusted_center[0], adjusted_center[1]), 
                                      (adjusted_center[0] + 50, adjusted_center[1]), 
                                      (0, 0, 255), 2, tipLength=0.3)
                    elif dx < -arrow_threshold:
                        cv2.arrowedLine(frame, (adjusted_center[0], adjusted_center[1]), 
                                      (adjusted_center[0] - 50, adjusted_center[1]), 
                                      (0, 0, 255), 2, tipLength=0.3)
                    
                    # 注意：摄像头倒置，dy方向也要反转
                    if dy > arrow_threshold:
                        cv2.arrowedLine(frame, (adjusted_center[0], adjusted_center[1]), 
                                      (adjusted_center[0], adjusted_center[1] - 50),  # 上移
                                      (0, 0, 255), 2, tipLength=0.3)
                    elif dy < -arrow_threshold:
                        cv2.arrowedLine(frame, (adjusted_center[0], adjusted_center[1]), 
                                      (adjusted_center[0], adjusted_center[1] + 50),  # 下移
                                      (0, 0, 255), 2, tipLength=0.3)

            # 检查定位成功条件
            success_condition = (
                presence and 
                SUCCESS_RADIUS[0] <= r <= SUCCESS_RADIUS[1] and
                abs(dx) < SUCCESS_DISTANCE and
                abs(dy) < SUCCESS_DISTANCE
            )
            
            if success_condition:
                success_count += 1
                success_text = f"Positioned! ({success_count}/{SUCCESS_THRESHOLD})"
                cv2.putText(frame, success_text, (RES_WIDTH//2 - 100, FRAME_HEIGHT//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # 连续满足条件才发送成功消息
                if success_count >= SUCCESS_THRESHOLD and not position_success_sent:
                    # 发送成功消息但不退出程序
                    success_msg = pack_system_message("POSITION_SUCCESS")
                    try:
                        tx_queue.put(success_msg)
                        position_success_sent = True
                        laser_guided_mode = True
                        print("[SUCCESS] Ball positioned correctly. Entering laser guided mode.")
                    except Full:
                        print("[WARNING] Serial queue full when sending success message")
            else:
                success_count = 0

            # 在激光引导模式下，只发送位置信息
            if not laser_guided_mode:
                # 传输速率调节（检测到球时提高频率）
                MIN_DT = MIN_DT_FOUND if presence else MIN_DT_LOST

                # 发送数据（避免频繁发送相同数据）
                if (now - last_send_time >= MIN_DT and 
                    (presence != last_presence or 
                     approach_flag != last_approach_flag or 
                     abs(dx) > 5 or abs(dy) > 5)):
                    
                    msg = pack_frame_message(presence, dx, dy, r, approach_flag)
                    try:
                        tx_queue.put_nowait(msg)
                    except Full:
                        print("[WARNING] Serial queue full, skipping frame message")
                    last_send_time = now
                    last_presence = presence
                    last_approach_flag = approach_flag

            # --- 显示处理 ---
            # 1. 计算目标框尺寸（使用缩放比例）
            TARGET_BOX_SIZE = int(APPROACH_RADIUS * 2 * TARGET_BOX_SCALE)
            
            # 2. 绘制调整后的目标框（使用缩放后的尺寸）
            cv2.rectangle(frame,
                (adjusted_center[0] - TARGET_BOX_SIZE//2, adjusted_center[1] - TARGET_BOX_SIZE//2),
                (adjusted_center[0] + TARGET_BOX_SIZE//2, adjusted_center[1] + TARGET_BOX_SIZE//2),
                (0, 165, 255), 2)
            
            # 3. 绘制两个中心点（原始和调整后）
            cv2.circle(frame, FRAME_CENTER, 5, (255, 0, 0), -1)  # 原始中心(蓝色)
            cv2.circle(frame, adjusted_center, 5, (255, 255, 0), -1)  # 调整后中心(青色)
            
            # 4. 添加偏移量说明文本
            cv2.putText(frame, f"Vertical Offset: {offset_y}px", 
                       (10, frame.shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)
            
            # 5. 绘制中心线
            cv2.line(frame, (0, adjusted_center[1]), (RES_WIDTH, adjusted_center[1]), (0, 255, 255), 1)
            
            # 6. 绘制成功条件区域
            cv2.circle(frame, adjusted_center, SUCCESS_DISTANCE, (0, 255, 0), 1)
            
            # 7. 绘制动态中心区域（虚线矩形）
            cv2.rectangle(frame,
                (adjusted_center[0] - center_threshold, adjusted_center[1] - center_threshold),
                (adjusted_center[0] + center_threshold, adjusted_center[1] + center_threshold),
                (100, 100, 255), 1, cv2.LINE_AA)
            
            # 显示状态信息
            cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # 显示当前模式
            mode_text = "Laser Guided" if laser_guided_mode else "Visual Tracking"
            cv2.putText(frame, f"Mode: {mode_text}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # 显示目标框大小和中心阈值信息
            cv2.putText(frame, f"Target Box: {TARGET_BOX_SCALE:.1f}x", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.putText(frame, f"Center Threshold: {center_threshold}px", (10, 140),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            
            # 串口状态
            status_color = (0, 255, 0) if ser and ser.is_open else (0, 0, 255)
            cv2.putText(frame, f"Serial: {'OK' if ser and ser.is_open else 'ERROR'}",
                        (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            # 显示检测数据
            if presence:
                coords_text = f"DX: {dx:.1f}, DY: {dy:.1f}, R: {r:.1f}"
                cv2.putText(frame, coords_text, (RES_WIDTH-300, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                app_text = f"Approach: {'YES' if approach_flag else 'NO'}"
                cv2.putText(frame, app_text, (RES_WIDTH-300, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0) if approach_flag else (0, 0, 255), 2)
                
                # 显示成功条件状态
                success_status = "SUCCESS" if success_condition else "Positioning"
                color = (0, 255, 0) if success_condition else (0, 255, 255)
                cv2.putText(frame, f"Status: {success_status}", 
                           (RES_WIDTH-300, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 激光引导模式显示
            if laser_guided_mode:
                cv2.putText(frame, "LASER GUIDANCE ACTIVE", (RES_WIDTH//2 - 150, FRAME_HEIGHT//2 + 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # 显示图像
            cv2.imshow('Tennis Ball Detection', frame)
            cv2.imshow('Mask', mask)
            cv2.imshow('Parameters', np.zeros((100, 500, 3), dtype=np.uint8))  # 参数窗口占位

            # 更新FPS计数
            frame_count += 1
            if frame_count >= 30:
                fps = frame_count / (time.time() - fps_start)
                fps_text = f"FPS: {fps:.1f}"
                fps_start = time.time()
                frame_count = 0

            # 检查退出键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):  # 重置定位成功状态
                position_success_sent = False
                laser_guided_mode = False
                success_count = 0
                print("[INFO] Reset position success state")

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
    except Exception as e:
        print(f"[MAIN ERROR] {e}")
    finally:
        cleanup(picam2, ser, t)

if __name__ == '__main__':
    main()