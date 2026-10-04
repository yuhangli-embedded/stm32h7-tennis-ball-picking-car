from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
import imutils
from threading import Thread
from queue import Queue
import os
import struct  # 新增：用于打包二进制数据

# === 配置参数 ===
SERIAL_PORT  = '/dev/serial0'
BAUD_RATE    = 115200
EST_BALL_RADIUS = 43                
TARGET_RADIUS   = 80                
CIRC_THRESH  = 0.5                  
MIN_RAD      = int(EST_BALL_RADIUS * 0.5)
MAX_RAD      = int(EST_BALL_RADIUS * 1.5)
RES_WIDTH    = 640                  
FRAME_HEIGHT = 480                  
FRAME_CENTER = (RES_WIDTH // 2, FRAME_HEIGHT // 2)

# 垂直偏移量补偿
VERTICAL_OFFSET = 34

# 串口消息队列
QUEUE_MAX    = 10

# 记录程序启动时间
start_time = time.time()
tx_queue = Queue(maxsize=QUEUE_MAX)

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
        if not ser.isOpen():
            print(f"[ERROR] Serial port {SERIAL_PORT} failed to open")
            return None
        print(f"[INFO] Opened serial {SERIAL_PORT} @ {BAUD_RATE}bps")
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
                    # 调试打印修改为十六进制输出，方便排查二进制协议包
                    # print(f"[SERIAL] Sent: {msg.hex()}") 
                    break
                except Exception as e:
                    print(f"[SERIAL ERROR] Attempt {retry+1}: {e}")
                    time.sleep(0.05)
                    retry += 1
        except:
            pass
    if ser and ser.isOpen():
        try:
            ser.close()
        except:
            pass
    print("[SERIAL] Worker thread exited")

# ================= 核心修改区 1 =================
def pack_binary_data(presence, dx=0, dy=0, radius=0):
    """
    将数据打包为 10 字节的定长二进制帧
    协议格式: [帧头0x55] [帧头0xAA] [存在标志1/0] [dx低] [dx高] [dy低] [dy高] [r低] [r高] [校验和]
    """
    header1 = 0x55
    header2 = 0xAA
    pres_byte = 1 if presence else 0
    # 限制数据范围并转为整型
    dx_int = int(dx)
    dy_int = int(dy)
    r_int = int(radius)

    # <BBbhhH 含义: 小端序(Little Endian), unsigned char, unsigned char, char, short, short, unsigned short
    # 占 1+1+1+2+2+2 = 9 bytes
    payload = struct.pack('<BBbhhH', header1, header2, pres_byte, dx_int, dy_int, r_int)
    
    # 计算校验和 (除去帧头的7个字节求和取低8位)
    checksum = sum(payload[2:]) & 0xFF
    
    # 拼装最终报文 (10 bytes)
    return payload + struct.pack('<B', checksum)
# ================================================

def cleanup(picam2, ser, t):
    print("[CLEANUP] Shutting down...")
    cv2.destroyAllWindows()
    if picam2:
        try: picam2.stop()
        except: pass
    if ser and ser.isOpen():
        try: ser.close()
        except: pass
    if t and t.is_alive():
        try:
            tx_queue.put(None)
            t.join(timeout=2.0)
        except: pass
    print("[CLEANUP] Completed")

def main():
    # 初始化摄像头
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (RES_WIDTH, FRAME_HEIGHT), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

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

    # 创建参数调整窗口
    cv2.namedWindow("Parameters")
    cv2.createTrackbar("LH", "Parameters", 67, 255, lambda x: None)
    cv2.createTrackbar("LS", "Parameters", 81, 255, lambda x: None)
    cv2.createTrackbar("LV", "Parameters", 69, 255, lambda x: None)
    cv2.createTrackbar("UH", "Parameters", 88, 255, lambda x: None)
    cv2.createTrackbar("US", "Parameters", 255, 255, lambda x: None)
    cv2.createTrackbar("UV", "Parameters", 255, 255, lambda x: None)
    cv2.createTrackbar("OffsetY", "Parameters", VERTICAL_OFFSET + 50, 100, lambda x: None)
    
    adjusted_center = (FRAME_CENTER[0], FRAME_CENTER[1] + VERTICAL_OFFSET)
    
    print(f"[INFO] Using vertical offset: {VERTICAL_OFFSET}px")
    print("[INFO] Starting detection loop. Press 'q' to quit.")
    
    # 定时发送控制
    last_tx_time = 0
    TX_INTERVAL = 0.05 # 50ms发一次，即20Hz，足够STM32平滑控制

    try:
        while True:
            # 读取滑动条值... (保持不变)
            lh = cv2.getTrackbarPos("LH", "Parameters")
            ls = cv2.getTrackbarPos("LS", "Parameters")
            lv = cv2.getTrackbarPos("LV", "Parameters")
            uh = cv2.getTrackbarPos("UH", "Parameters")
            us = cv2.getTrackbarPos("US", "Parameters")
            uv = cv2.getTrackbarPos("UV", "Parameters")
            offset_y = cv2.getTrackbarPos("OffsetY", "Parameters") - 50
            
            adjusted_center = (FRAME_CENTER[0], FRAME_CENTER[1] + offset_y)
            
            frame = picam2.capture_array()
            if frame is None:
                continue
            
            frame = imutils.resize(frame, width=RES_WIDTH)
            blurred = cv2.medianBlur(frame, 5)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (lh, ls, lv), (uh, us, uv))
            
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=1)
            
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            
            presence, dx, dy, r = False, 0.0, 0.0, 0.0
            best_contour = None
            best_circ = 0.0

            if cnts:
                for c in cnts:
                    area = cv2.contourArea(c)
                    if area < 100: continue
                    
                    peri = cv2.arcLength(c, True)
                    circ = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    
                    if MIN_RAD <= radius <= MAX_RAD and circ > CIRC_THRESH:
                        if circ > best_circ:
                            best_circ = circ
                            best_contour = c

            # ================= 核心修改区 2 =================
            if best_contour is not None:
                (x, y), radius = cv2.minEnclosingCircle(best_contour)
                cx, cy = x, y
                
                # 计算位置偏差
                dx = adjusted_center[0] - cx
                dy = adjusted_center[1] - cy
                r = radius
                presence = True

                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            
            # 无论是否找到球，按照固定频率（如 20Hz）将当前视觉数据发给 STM32
            current_time = time.time()
            if current_time - last_tx_time >= TX_INTERVAL:
                bin_msg = pack_binary_data(presence, dx, dy, r)
                if not tx_queue.full():
                    tx_queue.put(bin_msg)
                last_tx_time = current_time
            # ================================================

            # --- 显示处理 --- (简化了之前繁琐的状态机文字绘制，只保留坐标)
            cv2.circle(frame, FRAME_CENTER, 5, (255, 0, 0), -1) 
            cv2.circle(frame, adjusted_center, 5, (255, 255, 0), -1) 
            
            if presence:
                coords_text = f"DX: {dx:.1f}, DY: {dy:.1f}, R: {r:.1f}"
                cv2.putText(frame, coords_text, (RES_WIDTH-300, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                cv2.putText(frame, "NO BALL", (RES_WIDTH-150, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 更新FPS计数
            frame_count += 1
            if frame_count >= 30:
                fps = frame_count / (time.time() - fps_start)
                fps_text = f"FPS: {fps:.1f}"
                fps_start = time.time()
                frame_count = 0
            cv2.putText(frame, fps_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow('Tennis Ball Detection', frame)
            cv2.imshow('Mask', mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
    except Exception as e:
        print(f"[MAIN ERROR] {e}")
    finally:
        cleanup(picam2, ser, t)

if __name__ == '__main__':
    main()