/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Four-wheel PI speed control test for STM32H743
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
/* Includes */
#include "main.h"

/* USER CODE BEGIN Includes */
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Reserved search parameters */
#define VL53L1X_DEV_ADDR  0x52 << 1  

#define STATE_SEARCH        0  
#define STATE_CENTERING     1  
#define STATE_APPROACH      2  
#define STATE_LASER_GUIDED  3  
#define STATE_SUCCESS       4  

#define CENTER_THRESHOLD   20       
#define LASER_TARGET_MM    200      
#define LASER_MIN_MM       150      
#define LASER_START_MM     500      
#define FORWARD_SPEED      700      
#define TURN_SPEED         600      
#define SLOW_SPEED         400      
#define CENTER_TIMEOUT     5000     
#define APPROACH_TIMEOUT   10000    
#define MOTOR_PWM_60       600      

/* Reserved vision data */
typedef struct {
    uint32_t timestamp;      
    uint8_t presence;        
    float dx, dy;            
    float radius;            
    uint8_t approach_flag;   
} VisionData;
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* PID controller */
typedef struct {
    float Kp;                
    float Ki;                
    float Kd;                
    float error_sum;         
    float last_error;        
    float max_out;           
    float integral_limit;    
} PID_Controller;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Peripheral handles */
I2C_HandleTypeDef hi2c1;  

TIM_HandleTypeDef htim1;  
TIM_HandleTypeDef htim3;  
TIM_HandleTypeDef htim4;  
TIM_HandleTypeDef htim5;  
TIM_HandleTypeDef htim8;  

UART_HandleTypeDef huart1; 
UART_HandleTypeDef huart2; 

/* USER CODE BEGIN PV */
/* Reserved vision and state variables */
VisionData currentVision = {0};       
uint16_t distance_mm = 0;             
uint8_t dataReady = 0;                
char uartBuf[128];                    

uint8_t state = STATE_SEARCH;         
uint32_t last_state_change = 0;       
uint32_t last_print_time = 0;         
uint32_t last_vision_print = 0;       
uint32_t last_distance_print = 0;     

uint8_t debug_enabled = 1;            
uint8_t position_success_received = 0;

/* Reserved binary packet */
#pragma pack(push, 1)                 
typedef struct {
    uint8_t  header1;    
    uint8_t  header2;    
    uint8_t  presence;   
    int16_t  dx;         
    int16_t  dy;         
    uint16_t radius;     
    uint8_t  checksum;   
} VisionPacket;         
#pragma pack(pop)                     

uint8_t rxByte;               
uint8_t rxBuffer[10];         
uint8_t rxIndex = 0;          

VisionPacket current_vision_data = {0};  
uint32_t last_vision_update_time = 0;    
/* USER CODE END PV */

/* Function declarations */
void SystemClock_Config(void);               
static void MPU_Config(void);                
static void MX_GPIO_Init(void);              
static void MX_I2C1_Init(void);              
static void MX_TIM1_Init(void);              
static void MX_TIM3_Init(void);              
static void MX_TIM4_Init(void);              
static void MX_TIM5_Init(void);              
static void MX_USART1_UART_Init(void);       
static void MX_USART2_UART_Init(void);       
static void MX_TIM8_Init(void);              

/* USER CODE BEGIN PFP */
void motor_stop(void);                       
float PID_Calc(PID_Controller *pid, float target, float measure); 
void set_chassis_motion(int16_t base_speed, float turn_comp);     
void update_state_machine(void);             
void print_debug_info(void);                 
void parseVisionData(uint8_t *data);         
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* PWM stop */
void motor_stop(void) { 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); 
}

/* Reserved yaw controller */
PID_Controller turn_pid = {
    .Kp = 2.5f,               
    .Ki = 0.0f,               
    .Kd = 1.2f,               
    .error_sum = 0.0f,        
    .last_error = 0.0f,       
    .max_out = 400.0f,        
    .integral_limit = 100.0f  
};


/* Wheel speed PI parameters */
PID_Controller pid_a = {1.5f, 0.2f, 0.0f, 0.0f, 0.0f, 999.0f, 2000.0f}; 

PID_Controller pid_b = {1.5f, 0.2f, 0.0f, 0.0f, 0.0f, 999.0f, 2000.0f}; 

PID_Controller pid_c = {1.8f, 0.3f, 0.0f, 0.0f, 0.0f, 999.0f, 2500.0f}; 

PID_Controller pid_d = {1.5f, 0.2f, 0.0f, 0.0f, 0.0f, 999.0f, 2000.0f};

/* PID calculation */
float PID_Calc(PID_Controller *pid, float target, float measure) {
    float error = target - measure;                               
    float p_out, i_out, d_out, total_out;

    p_out = pid->Kp * error;                                      

    pid->error_sum += error;                                      
    if (pid->error_sum > pid->integral_limit) pid->error_sum = pid->integral_limit;     
    if (pid->error_sum < -pid->integral_limit) pid->error_sum = -pid->integral_limit;   
    i_out = pid->Ki * pid->error_sum;                             

    d_out = pid->Kd * (error - pid->last_error);                  
    pid->last_error = error;                                      

    total_out = p_out + i_out + d_out;                            

    /* PWM deadband compensation */
    float deadband = 600.0f;                                      
    
    if (target > 0.5f) {                                          
        if (total_out > 0.0f) {
            total_out += deadband;       
        }
    } else if (target < -0.5f) {
         if (total_out < 0.0f) {
            total_out -= deadband;      
         }
    }

    if (total_out > pid->max_out) total_out = pid->max_out;       
    if (total_out < -pid->max_out) total_out = -pid->max_out;     

    return total_out;                                             
}

/* Legacy differential PWM control */
void set_chassis_motion(int16_t base_speed, float turn_comp) {
    int16_t pwm_left = base_speed + (int16_t)turn_comp;  
    int16_t pwm_right = base_speed - (int16_t)turn_comp; 

    if (pwm_left > 999) pwm_left = 999;
    if (pwm_left < -999) pwm_left = -999;
    if (pwm_right > 999) pwm_right = 999;
    if (pwm_right < -999) pwm_right = -999;

    if (pwm_left >= 0) {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_left); 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_left); 
    } else {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm_left); 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -pwm_left); 
    }

    if (pwm_right >= 0) {
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);    
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_right);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_right);
    } else {
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);  
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pwm_right);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -pwm_right);
    }
}

/* Independent wheel PWM control */
void set_4wheel_pwm(int16_t pwm_a, int16_t pwm_b, int16_t pwm_c, int16_t pwm_d) {
    if (pwm_a > 999) pwm_a = 999; if (pwm_a < -999) pwm_a = -999;
    if (pwm_b > 999) pwm_b = 999; if (pwm_b < -999) pwm_b = -999;
    if (pwm_c > 999) pwm_c = 999; if (pwm_c < -999) pwm_c = -999;
    if (pwm_d > 999) pwm_d = 999; if (pwm_d < -999) pwm_d = -999;

    if (pwm_a >= 0) {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);    
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_a);   
    } else {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);  
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm_a);  
    }

    if (pwm_b >= 0) {
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_SET);    
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_b);
    } else {
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_RESET);  
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pwm_b);
    }

    if (pwm_c >= 0) {
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_c);
    } else {
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -pwm_c);
    }

    if (pwm_d >= 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);    
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_d);
    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);    
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);  
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -pwm_d);
    }
}
/* USER CODE END 0 */

/* Main */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  MPU_Config(); 


  HAL_Init(); 

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config(); 

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Peripheral initialization */
  MX_GPIO_Init();        
  MX_I2C1_Init();        
  MX_TIM1_Init();        
  MX_TIM3_Init();        
  MX_TIM4_Init();        
  MX_TIM5_Init();        
  MX_USART1_UART_Init(); 
  MX_USART2_UART_Init(); 
  MX_TIM8_Init();        
  
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*)"System Initialized\r\n", 20, 100);
  
  HAL_UART_Receive_IT(&huart2, &rxByte, 1);
  
  /* PWM and encoder startup */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); 
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); 
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); 
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); 

  state = STATE_SEARCH; 
  last_state_change = HAL_GetTick(); 
  
  HAL_UART_Transmit(&huart1, (uint8_t*)"Starting tennis ball collection...\r\n", 36, 100);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
        HAL_Delay(100);
        
        /* Encoder sampling */
        int16_t speed5 = (int16_t)__HAL_TIM_GET_COUNTER(&htim5); 
        int16_t speed4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4); 
        int16_t speed8 = (int16_t)__HAL_TIM_GET_COUNTER(&htim8); 
        int16_t speed3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3); 
        
        __HAL_TIM_SET_COUNTER(&htim5, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        __HAL_TIM_SET_COUNTER(&htim8, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);

        /* Encoder direction correction */
        float actual_speed_a = - (float)speed5; 
        float actual_speed_b =   (float)speed4; 
        float actual_speed_c =   (float)speed8; 
        float actual_speed_d = - (float)speed3; 

        /* Wheel speed control */
        float target_speed = 170.0f; 

        int16_t pwm_a = (int16_t)PID_Calc(&pid_a, target_speed, actual_speed_a);
        int16_t pwm_b = (int16_t)PID_Calc(&pid_b, target_speed, actual_speed_b);
        int16_t pwm_c = (int16_t)PID_Calc(&pid_c, target_speed, actual_speed_c);
        int16_t pwm_d = (int16_t)PID_Calc(&pid_d, target_speed, actual_speed_d);

        set_4wheel_pwm(pwm_a, pwm_b, pwm_c, pwm_d);
        
        /* VOFA+ telemetry */
        char vofaBuf[128];
        int len = snprintf(vofaBuf, sizeof(vofaBuf), 
                           "%.1f,%.1f,%.1f,%.1f,%.1f\n", 
                           target_speed, actual_speed_a, actual_speed_b, actual_speed_c, actual_speed_d);
        HAL_UART_Transmit(&huart1, (uint8_t*)vofaBuf, len, 100); 
        
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* Peripheral configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

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

static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

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
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif  
