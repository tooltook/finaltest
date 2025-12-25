/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_can.h"
// #include "bsp_key.h"
#include <stdbool.h>

#include "bsp_led.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "pid.h"
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

/* USER CODE BEGIN PV */
#define MOTOR_IDX 0
extern rc_info_t rc;//接收rc遥控器的信息结构体
char buf[200];//串口通信相关
int count;//计数结构
extern moto_info_t motor_info[MOTOR_MAX_NUM];//定义了最大电机数量的结构体
int16_t led_cnt;//闪烁计数
pid_struct_t motor_pid[7];//储存pid控制参数
float target_speed;
uint16_t pwm_pulse = 1080;

static pid_struct_t pos_pid;
static uint16_t last_enc = 0;

static int32_t total_angle1 = 0;
static int32_t total_angle2 = 0;

static int32_t target_angle1 = 0;
static int32_t target_angle2 = 0;

float g_pid_output1 = 0.0f;   // 全局 PID 输出
float g_pid_output2 = 0.0f;   // 全局 PID 输出

int32_t g_error1 = 0;
int32_t g_error2 = 0;

static int32_t lock_position1 = 0;   // 锁定的目标位置（松手时记录）
static int32_t lock_position2 = 0;   // 锁定的目标位置（松手时记录）

static bool is_in_manual_move1 = false; // 是否正在用摇杆移动
static bool is_in_manual_move2 = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void position_control_init(void);
void position_control_loop1(void);
void position_control_loop2(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void position_control_init(void)
{
  while (motor_info[MOTOR_IDX].rotor_angle == 0) {
    HAL_Delay(1);
  }

  last_enc = motor_info[MOTOR_IDX].rotor_angle;
  total_angle1 = 0;
  total_angle2 = 0;

  angle_update_total(&total_angle1, motor_info[MOTOR_IDX].rotor_angle, &last_enc);
  angle_update_total(&total_angle2, motor_info[MOTOR_IDX].rotor_angle, &last_enc);

  // 初始化锁定位置为当前位置
  lock_position1 = total_angle1;
  lock_position2 = total_angle2;

  is_in_manual_move1 = false;
  is_in_manual_move2 = false;

  // 提高 Kp 让力矩足够（关键！）
  pid_init(&pos_pid,
           70.0f,   // Kp: 60~80 起步
           0.0f,
           1.2f,    // Kd: 抑制抖动
           1000.0f,
           12000.0f
  );
}


void position_control_loop1(void)
{
  static uint32_t last_tick = 0;
  if (HAL_GetTick() - last_tick < 10) return;
  last_tick = HAL_GetTick();

  // 更新多圈角度
  angle_update_total(&total_angle1, motor_info[MOTOR_IDX].rotor_angle, &last_enc);

  const int16_t DEAD_ZONE = 20;
  const int32_t ANGLE_STEP = 200;

  // 判断是否处于“主动控制”状态
  if (rc.sw1 == 2 && abs(rc.ch2) > DEAD_ZONE) {
    // 正在用摇杆移动
    is_in_manual_move1 = true;
    target_angle1 += (int32_t)rc.ch2 * ANGLE_STEP / 1000;
    // 实时更新锁定点（可选，也可只在松手时更新）
    lock_position1 = target_angle1;
  } else {
    // 不在主动控制（松手 或 不在档位）
    if (is_in_manual_move1) {
      // 刚松手：记录当前位置作为锁定目标
      lock_position1 = total_angle1;
      is_in_manual_move1 = false;
    }
    // 无论是否刚松手，只要不主动控制，就使用锁定位置
    target_angle1 = lock_position1;
  }

  // PID 计算
  g_error1 = target_angle1 - total_angle1;
  g_pid_output1 = pid_calc(&pos_pid, (float)target_angle1, (float)total_angle1);

  // 发送指令
  set_motor_voltage(0, (int16_t)g_pid_output1, 0,
                       0, 0);
}



void position_control_loop2(void)
{
  static uint32_t last_tick = 0;
  if (HAL_GetTick() - last_tick < 10) return;
  last_tick = HAL_GetTick();

  // 更新多圈角度
  angle_update_total(&total_angle2, motor_info[MOTOR_IDX].rotor_angle, &last_enc);

  const int16_t DEAD_ZONE = 20;
  const int32_t ANGLE_STEP = 200;

  // 判断是否处于“主动控制”状态
  if (rc.sw1 == 2 && abs(rc.ch1) > DEAD_ZONE) {
    // 正在用摇杆移动
    is_in_manual_move2 = true;
    target_angle2 += (int32_t)rc.ch1 * ANGLE_STEP / 1000;
    // 实时更新锁定点（可选，也可只在松手时更新）
    lock_position2 = target_angle2;
  } else {
    // 不在主动控制（松手 或 不在档位）
    if (is_in_manual_move2) {
      // 刚松手：记录当前位置作为锁定目标
      lock_position2 = total_angle2;
      is_in_manual_move2 = false;
    }
    // 无论是否刚松手，只要不主动控制，就使用锁定位置
    target_angle2 = lock_position2;
  }

  // PID 计算
  g_error2 = target_angle2 - total_angle2;
  g_pid_output2 = pid_calc(&pos_pid, (float)target_angle2, (float)total_angle2);

  // 发送指令
  set_motor_voltage(0, 0, (int16_t)g_pid_output2,
                       0, 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_CAN1_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    dbus_uart_init();


    HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin|POWER2_CTRL_Pin|POWER3_CTRL_Pin|POWER4_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
    pwm_init();                              // start pwm output
    can_user_init(&hcan1);                   // config can filter, start can

    position_control_init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {


      /* led blink */
      led_cnt ++;
      if (led_cnt == 250)
      {
        led_cnt = 0;
        LED_RED_TOGGLE(); //blink cycle 500ms
      }

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      HAL_Delay(1);
      position_control_loop1();
      position_control_loop2();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
