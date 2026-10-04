/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program for tennis ball collection robot
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define VL53L1X_DEV_ADDR  0x52      // 7-bit address << 1

// 状态定义
#define STATE_SEARCH        0  // 搜索状态：机器人正在旋转寻找网球
#define STATE_CENTERING     1  // 居中状态：机器人检测到网球后，调整位置使其在视野中心
#define STATE_APPROACH      2  // 接近状态：机器人向网球移动，保持网球在视野中心
#define STATE_LASER_GUIDED  3  // 激光引导状态：使用激光测距传感器进行精确接近
#define STATE_SUCCESS       4  // 成功状态：机器人已到达目标位置，收集网球

// 参数配置
#define CENTER_THRESHOLD   20       // 中心位置阈值(像素)
#define LASER_TARGET_MM    200      // 激光测距目标距离(mm)
#define LASER_MIN_MM       150      // 最小安全距离(mm)
#define LASER_START_MM     500      // 开始激光引导的距离(mm)
#define FORWARD_SPEED      700      // 前进速度(0-999)
#define TURN_SPEED         600      // 转向速度(0-999)
#define SLOW_SPEED         400      // 慢速前进速度
#define CENTER_TIMEOUT     5000     // 居中状态超时(ms)
#define APPROACH_TIMEOUT   10000    // 接近状态超时(ms)
#define LASER_START_MM     500   // 开始激光引导的距离(mm)
#define LASER_TARGET_MM    200   // 目标距离(mm)
#define LASER_MIN_MM       150   // 最小安全距离(mm)
#define MOTOR_PWM_60       600   // 60% PWM速度

// 视觉数据结构
typedef struct {
    uint32_t timestamp;
    uint8_t presence;      // 目标存在标志
    float dx, dy;          // 位置偏移
    float radius;          // 目标半径
    uint8_t approach_flag; // 允许接近标志
} VisionData;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//定时器与串口等句柄
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;   // PWM for EN pins
TIM_HandleTypeDef htim2;   // Encoder C
TIM_HandleTypeDef htim3;   // Encoder D
TIM_HandleTypeDef htim4;   // Encoder A
TIM_HandleTypeDef htim5;   // Encoder B

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// 全局变量
VisionData currentVision = {0};       //当前视觉数据
uint16_t distance_mm = 0;							//激光测距的距离
uint8_t dataReady = 0;								//数据就绪标志
char uartBuf[128];                    //串口输出缓冲区

// 串口接收相关变量
uint8_t rxBuffer[128];  //串口接收缓冲区
uint8_t rxByte;         //接收字节
uint16_t rxIndex = 0;   //接收索引
uint8_t frameStarted = 0;   //帧开始标志

// 状态机变量
uint8_t state = STATE_SEARCH;         // 当前状态
uint32_t last_state_change = 0;       // 上次状态改变时间
uint32_t last_print_time = 0;         // 上次打印时间
uint32_t last_vision_print = 0;       // 上次视觉打印时间
uint32_t last_distance_print = 0;     // 上次距离打印时间

uint8_t debug_enabled = 1;            // 调试使能标志
uint8_t position_success_received = 0;// 定位成功标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//声明系统初始化函数和用户自定义函数
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
// 函数声明
void motor_stop(void);
void motor_forward(uint16_t speed);
void motor_backward(uint16_t speed);
void motor_turn_left(uint16_t speed);
void motor_turn_right(uint16_t speed);
void parseVisionData(uint8_t *data);
void update_state_machine(void);
void print_debug_info(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief 解析视觉数据
  * @param data 收到的视觉数据帧
  */
void parseVisionData(uint8_t *data) {
    // 解析字符串格式数据；提取时间戳、位置偏移、半径等信息；检测"POSITION_SUCCESS"消息
    char *token = strtok((char*)data, ",");
    
    // 临时变量
    uint32_t t = 0;
    uint8_t pres = 0;
    float dx = 0, dy = 0, r = 0;
    uint8_t app = 0;
    
    while (token != NULL) {
        if (strstr(token, "t=")) {
            t = atoi(token + 2);
        } 
        else if (strstr(token, "pres=")) {
            pres = atoi(token + 5);
        } 
        else if (strstr(token, "dx=")) {
            dx = atof(token + 3);
        } 
        else if (strstr(token, "dy=")) {
            dy = atof(token + 3);
        } 
        else if (strstr(token, "r=")) {
            r = atof(token + 2);
        } 
        else if (strstr(token, "app=")) {
            app = atoi(token + 4);
        }
        // 检测定位成功消息
        else if (strstr(token, "SYSTEM") && strstr(token, "POSITION_SUCCESS")) {
            position_success_received = 1;
            if (debug_enabled) {
                HAL_UART_Transmit(&huart1, (uint8_t*)"[SUCCESS] Position success received!\r\n", 38, 100);
            }
        }
        token = strtok(NULL, ",");
    }
    
    // 更新视觉数据结构
    currentVision.timestamp = t;
    currentVision.presence = pres;
    currentVision.dx = dx;
    currentVision.dy = dy;
    currentVision.radius = r;
    currentVision.approach_flag = app;
}

/**
  * @brief USART2接收中断回调函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // 重新使能下一次中断
    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
        
        if (rxByte == '<') {
            frameStarted = 1;
            rxIndex = 0;
            rxBuffer[rxIndex++] = rxByte;
        }
        else if (frameStarted) {
            if (rxIndex < sizeof(rxBuffer)-1) {
                rxBuffer[rxIndex++] = rxByte;
                // 检测到完整帧尾 ">\n"
                if (rxByte == '\n' && rxIndex >= 2 && rxBuffer[rxIndex-2] == '>') {
                    rxBuffer[rxIndex] = '\0'; // 添加字符串结束符
                    
                    // 调试输出
                    if (debug_enabled) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"[VISION RAW] ", 12, 50);
                        HAL_UART_Transmit(&huart1, rxBuffer, rxIndex, 100);
                        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 50);
                    }
                    
                    parseVisionData(rxBuffer);
                    frameStarted = 0;
                }
            } else {
                // 缓冲区溢出，重置
                frameStarted = 0;
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[WARN] Vision buffer overflow!\r\n", 31, 100);
                }
            }
        }
    }
}

/**
  * @brief 电机控制函数
  */
void motor_stop(void) { //关闭所有电机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); // D
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // C
}

/**
  * @brief 控制小车前进
  * @param speed PWM速度值 (0-999)
  */
/**
  * @brief 控制小车前进 (修正方向)
  * @param speed PWM速度值 (0-999)
  */
void motor_forward(uint16_t speed) {
    // 左侧电机 (A和C) - 正转 (修正方向)
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);   // A_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);      // A_IN2
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);    // C_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET);      // C_IN2
    
    // 右侧电机 (B和D) - 正转 (修正方向)
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);    // B_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_SET);      // B_IN2
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);     // D_IN1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);       // D_IN2
    
    // 设置PWM速度
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); // A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed); // B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed); // D
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed); // C
}

/**
  * @brief 控制小车后退 (修正方向)
  * @param speed PWM速度值 (0-999)
  */
void motor_backward(uint16_t speed) {
    // 左侧电机 (A和C) - 反转 (修正方向)
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_SET);      // A_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);     // A_IN2
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);       // C_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET);     // C_IN2
    
    // 右侧电机 (B和D) - 反转 (修正方向)
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);       // B_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_RESET);     // B_IN2
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);       // D_IN1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);     // D_IN2
    
    // 设置PWM速度
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); // A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed); // B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed); // D
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed); // C
}

/**
  * @brief 控制小车左转 (修正方向)
  * @param speed PWM速度值 (0-999)
  */
void motor_turn_left(uint16_t speed) {
    // 左侧后退 (修正方向)
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_SET);      // A_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);     // A_IN2
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);       // C_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET);     // C_IN2
    
    // 右侧前进 (修正方向)
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);    // B_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_SET);      // B_IN2
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);     // D_IN1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);       // D_IN2
    
    // 设置PWM速度
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); // A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed); // B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed); // D
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed); // C
}

/**
  * @brief 控制小车右转 (修正方向)
  * @param speed PWM速度值 (0-999)
  */
void motor_turn_right(uint16_t speed) {
    // 左侧前进 (修正方向)
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);   // A_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);      // A_IN2
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);    // C_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET);      // C_IN2
    
    // 右侧后退 (修正方向)
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);       // B_IN1
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_RESET);     // B_IN2
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);       // D_IN1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);     // D_IN2
    
    // 设置PWM速度
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); // A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed); // B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed); // D
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed); // C
}

/**
  * @brief 更新状态机逻辑
  */
/* USER CODE BEGIN 0 */
/**
  * @brief 更新状态机逻辑
  */
void update_state_machine(void) {
    uint32_t current_time = HAL_GetTick();
    static uint32_t last_ball_time = 0;
    static uint32_t laser_start_time = 0;
    
    // 检查是否收到定位成功消息 - 最高优先级
    if (position_success_received) {
        state = STATE_LASER_GUIDED;
        last_state_change = current_time;
        laser_start_time = current_time;  // 记录进入激光引导的时间
        position_success_received = 0;    // 重置标志
        motor_stop();  // 确保停止
        
        if (debug_enabled) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] Transition to LASER_GUIDED (from position success)\r\n", 55, 100);
        }
        return;  // 立即处理状态切换
    }
    
    // 检测网球是否丢失（超过500ms未更新）
    if (current_time - last_ball_time > 500 && state != STATE_SEARCH && state != STATE_LASER_GUIDED) {
        if (debug_enabled) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] Ball lost, returning to SEARCH\r\n", 40, 100);
        }
        state = STATE_SEARCH;
        motor_stop();
        return;
    }
    
    // 更新最后检测到网球的时间
    if (currentVision.presence) {
        last_ball_time = current_time;
    }
    
    switch (state) {
        case STATE_SEARCH:
            // 未检测到网球，原地旋转
            if (!currentVision.presence) {
                motor_turn_left(TURN_SPEED);
                
                // 状态超时处理
                if (current_time - last_state_change > 10000) {
                    motor_turn_right(TURN_SPEED); // 改变旋转方向
                    last_state_change = current_time;
                }
            } 
            // 检测到网球，进入居中状态
            else {
                state = STATE_CENTERING;
                last_state_change = current_time;
                motor_stop();
                HAL_Delay(100); // 短暂停止
                
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] Transition to CENTERING\r\n", 32, 100);
                }
            }
            break;
            
        case STATE_CENTERING:
            // 网球丢失，返回搜索状态
            if (!currentVision.presence) {
                state = STATE_SEARCH;
                motor_stop();
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] Ball lost, back to SEARCH\r\n", 35, 100);
                }
            } 
            // 状态超时处理
            else if (current_time - last_state_change > CENTER_TIMEOUT) {
                state = STATE_SEARCH;
                motor_stop();
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] CENTERING timeout\r\n", 27, 100);
                }
            }
            // 调整位置使网球居中
            else {
                // X方向调整（左右）
                if (fabs(currentVision.dx) > CENTER_THRESHOLD) {
                    if (currentVision.dx > 0) {
                        motor_turn_right(TURN_SPEED);
                    } else {
                        motor_turn_left(TURN_SPEED);
                    }
                } 
                // Y方向调整（前后）
                else if (fabs(currentVision.dy) > CENTER_THRESHOLD) {
                    if (currentVision.dy > 0) {
                        motor_backward(SLOW_SPEED); // 网球偏下，后退
                    } else {
                        motor_forward(SLOW_SPEED); // 网球偏上，前进
                    }
                }
                // 网球已居中
                else {
                    state = STATE_APPROACH;
                    last_state_change = current_time;
                    
                    if (debug_enabled) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] Transition to APPROACH\r\n", 31, 100);
                    }
                }
            }
            break;
            
        case STATE_APPROACH:
            // 网球丢失，返回搜索状态
            if (!currentVision.presence) {
                state = STATE_SEARCH;
                motor_stop();
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] Ball lost, back to SEARCH\r\n", 35, 100);
                }
            } 
            // 状态超时处理
            else if (current_time - last_state_change > APPROACH_TIMEOUT) {
                state = STATE_SEARCH;
                motor_stop();
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] APPROACH timeout\r\n", 26, 100);
                }
            }
            // 控制逻辑：r<43时前进，dx/dy不在0附近时左右调整
            else {
                // 1. 位置调整优先：如果dx超过中心阈值，进行左右调整
                if (fabs(currentVision.dx) > CENTER_THRESHOLD) {
                    if (currentVision.dx > 0) {
                        motor_turn_right(TURN_SPEED); // 球在右边，向右转
                        if (debug_enabled) {
                            HAL_UART_Transmit(&huart1, (uint8_t*)"[APPROACH] Turning right\r\n", 25, 100);
                        }
                    } else {
                        motor_turn_left(TURN_SPEED);  // 球在左边，向左转
                        if (debug_enabled) {
                            HAL_UART_Transmit(&huart1, (uint8_t*)"[APPROACH] Turning left\r\n", 24, 100);
                        }
                    }
                } 
                // 2. 前进条件：如果位置基本居中且r<43，前进
                else if (currentVision.radius < 43) {
                    motor_forward(FORWARD_SPEED);
                    if (debug_enabled) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"[APPROACH] Moving forward (r<43)\r\n", 33, 100);
                    }
                }
                // 3. 当r>=43时，停止并等待定位成功消息
                else {
                    motor_stop();
                    if (debug_enabled) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"[APPROACH] Stopped (r>=43, waiting for success)\r\n", 47, 100);
                    }
                    
                    // 额外安全机制：如果等待超过5秒还没有收到成功消息，返回搜索
                    if (current_time - last_state_change > 5000) {
                        state = STATE_SEARCH;
                        if (debug_enabled) {
                            HAL_UART_Transmit(&huart1, (uint8_t*)"[APPROACH] Timeout waiting for success message\r\n", 48, 100);
                        }
                    }
                }
            }
            break;
            
        case STATE_LASER_GUIDED:
            // 确保激光测距已启动
            if (laser_start_time == 0) {
                laser_start_time = current_time;
            }
            
            // 读取激光距离数据
            if (VL53L1X_CheckForDataReady(VL53L1X_DEV_ADDR, &dataReady) == 0 && dataReady) {
                VL53L1X_GetDistance(VL53L1X_DEV_ADDR, &distance_mm);
                VL53L1X_ClearInterrupt(VL53L1X_DEV_ADDR);
                
                if (debug_enabled) {
                    int len = snprintf(uartBuf, sizeof(uartBuf), 
                                      "[LASER] Distance: %d mm\r\n", distance_mm);
                    HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, len, 100);
                }
                
                // 检查距离范围并控制电机
                if (distance_mm >= LASER_MIN_MM && distance_mm <= LASER_START_MM) {
                    // 前进逻辑 - 确保小车前进
                    
                    // 左侧电机 (A和C) - 正转
                    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);   // A_IN1
                    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);  // A_IN2
                    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);    // C_IN1
                    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET);  // C_IN2
                    
                    // 右侧电机 (B和D) - 正转
                    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);    // B_IN1
                    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_SET);  // B_IN2
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);    // D_IN1
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);  // D_IN2
                    
                    // 设置PWM速度 (60%)
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_PWM_60); // A
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MOTOR_PWM_60); // B
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MOTOR_PWM_60); // D
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MOTOR_PWM_60); // C
                    
                    if (debug_enabled) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"[LASER] Moving forward\r\n", 22, 100);
                    }
                } else {
                    // 停止所有电机
                    motor_stop();
                    
                    if (debug_enabled) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"[LASER] Stopped\r\n", 16, 100);
                    }
                    
                    // 达到目标距离，停止
                    if (distance_mm > 0 && distance_mm <= LASER_TARGET_MM) {
                        state = STATE_SUCCESS;
                        if (debug_enabled) {
                            HAL_UART_Transmit(&huart1, (uint8_t*)"[SUCCESS] Reached target distance (200mm)!\r\n", 44, 100);
                        }
                    }
                }
            } else {
                // 没有数据时停止
                motor_stop();
            }
            
            // 状态超时处理（防止卡在激光引导状态）
            if (current_time - laser_start_time > 15000) {  // 15秒超时
                state = STATE_SEARCH;
                motor_stop();
                laser_start_time = 0;
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] LASER_GUIDED timeout\r\n", 30, 100);
                }
            }
            break;
            
        case STATE_SUCCESS:
            // 成功状态，保持停止
            motor_stop();
            
            // 检测网球是否移动（重新出现）
            if (currentVision.presence && 
                (fabs(currentVision.dx) > CENTER_THRESHOLD || 
                 fabs(currentVision.dy) > CENTER_THRESHOLD)) {
                state = STATE_CENTERING;
                last_state_change = current_time;
                if (debug_enabled) {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"[STATE] Ball moved, returning to CENTERING\r\n", 41, 100);
                }
            }
            break;
    }
}

/**
  * @brief 打印调试信息
  */
void print_debug_info(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 打印视觉数据
    if (current_time - last_vision_print > 300) {
        int len = snprintf(uartBuf, sizeof(uartBuf), 
                          "[VISION] pres=%d, dx=%.1f, dy=%.1f, r=%.1f, app=%d\r\n",
                          currentVision.presence, 
                          currentVision.dx, 
                          currentVision.dy, 
                          currentVision.radius,
                          currentVision.approach_flag);
        HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, len, 100);
        last_vision_print = current_time;
    }
    
    // 打印激光测距数据
    if (current_time - last_distance_print > 300) {
        int len = snprintf(uartBuf, sizeof(uartBuf), 
                          "[LASER] Distance: %d mm\r\n", distance_mm);
        HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, len, 100);
        last_distance_print = current_time;
    }
    
    // 打印状态信息
    if (current_time - last_print_time > 1000) {
        const char* state_names[] = {
            "SEARCH", "CENTERING", "APPROACH", "LASER_GUIDED", "SUCCESS"
        };
        int len = snprintf(uartBuf, sizeof(uartBuf), 
                          "[STATE] Current: %s, Time: %lums\r\n",
                          state_names[state], current_time - last_state_change);
        HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, len, 100);
        last_print_time = current_time;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MPU_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*)"System Initialized\r\n", 20, 100);
  
  // 启用USART2接收中断（来自树莓派的数据）
  HAL_UART_Receive_IT(&huart2, &rxByte, 1);
  
  // 初始化VL53L1X传感器
  if (VL53L1X_SensorInit(VL53L1X_DEV_ADDR) != 0) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"VL53L1X Init Failed!\r\n", 22, 100);
  } else {
      VL53L1X_SetDistanceMode(VL53L1X_DEV_ADDR, 1); // 设置距离模式为短距离
      VL53L1X_SetTimingBudgetInMs(VL53L1X_DEV_ADDR, 50); // 设置测距周期为50ms
      if (VL53L1X_StartRanging(VL53L1X_DEV_ADDR) != 0) {
          HAL_UART_Transmit(&huart1, (uint8_t*)"VL53L1X Start Failed!\r\n", 23, 100);
      } else {
          HAL_UART_Transmit(&huart1, (uint8_t*)"VL53L1X Ready\r\n", 15, 100);
      }
  }
  
  // 启动PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  
  // 初始状态
  state = STATE_SEARCH;
  last_state_change = HAL_GetTick();
  
  HAL_UART_Transmit(&huart1, (uint8_t*)"Starting tennis ball collection...\r\n", 36, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. 处理激光测距数据
    if (VL53L1X_CheckForDataReady(VL53L1X_DEV_ADDR, &dataReady) == 0 && dataReady) {
        VL53L1X_GetDistance(VL53L1X_DEV_ADDR, &distance_mm);
        VL53L1X_ClearInterrupt(VL53L1X_DEV_ADDR);
    }
    
    // 2. 更新状态机
    update_state_machine();
    
    // 3. 打印调试信息
    if (debug_enabled) {
        print_debug_info();
    }
    
    HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PI8 PI0 PI2 PI4
                           PI6 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PH14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
