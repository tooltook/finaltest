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
extern rc_info_t rc;//遥控器解析过后的数据
char buf[200];
int count;
extern moto_info_t motor_info[MOTOR_MAX_NUM];//电机反馈数据。这里可以拿来用于做限位
int16_t led_cnt;
pid_struct_t motor_pid[7];
float target_speed;
uint16_t pwm_pulse = 1080;



#define MOTOR_NUM 2
#define MOTOR0_IDX 0
#define MOTOR1_IDX 1

static pid_struct_t pos_pid[MOTOR_NUM];

static int32_t total_angle[MOTOR_NUM] = {0};
static uint16_t last_enc[MOTOR_NUM] = {0};
static int32_t target_angle[MOTOR_NUM] = {0};
static int32_t lock_position[MOTOR_NUM] = {0};
static bool is_in_manual_move[MOTOR_NUM] = {false};

/* 上电校准得到的机械限位（单位必须和 total_angle 一致） */
static int32_t mech_min[MOTOR_NUM] = {-200000, -200000};
static int32_t mech_max[MOTOR_NUM] = { 200000,  200000};


float g_pid_output = 0.0f;   // 全局 PID 输出
int32_t g_error = 0;

static bool g_calibrated = false;





//这里的控制逻辑是为了实现上电校准，这里可以手动设置机械限位

/* 用于开关“边沿检测”，避免一秒记录几十次 */
static int8_t last_sw2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void position_control_init(void);
void position_control_loop(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void position_control_init(void)
{
  for (int i = 0; i < MOTOR_NUM; i++) {

    while (motor_info[i].rotor_angle == 0) HAL_Delay(1);

    if (!g_calibrated) {
      last_enc[i] = motor_info[i].rotor_angle;
      total_angle[i] = 0;
      angle_update_total(&total_angle[i], motor_info[i].rotor_angle, &last_enc[i]);
    } else {


    }

    target_angle[i] = total_angle[i];
    lock_position[i] = total_angle[i];
    is_in_manual_move[i] = false;

    pid_init(&pos_pid[i], 70.0f, 0.0f, 1.2f, 1000.0f, 12000.0f);
  }
}



void position_control_loop(void)
{
  static uint32_t last_tick = 0;
  if (HAL_GetTick() - last_tick < 10) return;
  last_tick = HAL_GetTick();

  const int16_t DEAD_ZONE = 20;
  const int32_t ANGLE_STEP = 800;

  int16_t ch_cmd[MOTOR_NUM] = { rc.ch2, rc.ch3 };

  int16_t out[MOTOR_NUM] = {0};

  for (int i = 0; i < MOTOR_NUM; i++) {
    /* 更新多圈角 */
    angle_update_total(&total_angle[i], motor_info[i].rotor_angle, &last_enc[i]);

    /* 手动 or 锁位逻辑 */
    if (rc.sw1 == 2 && abs(ch_cmd[i]) > DEAD_ZONE) {
      is_in_manual_move[i] = true;
      target_angle[i] += (int32_t)ch_cmd[i] * ANGLE_STEP / 1000;
      lock_position[i] = target_angle[i];
    } else {
      if (is_in_manual_move[i]) {
        lock_position[i] = total_angle[i];
        is_in_manual_move[i] = false;
      }
      target_angle[i] = lock_position[i];
    }

    /* 可选但强烈建议：目标角夹在限位内，防止目标本身越界 */
    if (target_angle[i] < mech_min[i]) target_angle[i] = mech_min[i];
    if (target_angle[i] > mech_max[i]) target_angle[i] = mech_max[i];

    /* PID */
    float pid_out = pid_calc(&pos_pid[i], (float)target_angle[i], (float)total_angle[i]);
    out[i] = (int16_t)pid_out;

    /* 输出层限位门禁：到限位且仍想继续越界 -> 停输出 */
    if (total_angle[i] <= mech_min[i] && target_angle[i] < total_angle[i]) out[i] = 0;
    if (total_angle[i] >= mech_max[i] && target_angle[i] > total_angle[i]) out[i] = 0;
  }

  /* 只控制两个电机，其它通道置0，避免误控 */
  set_motor_voltage(0, out[0], out[1], 0, 0);
}




static void update_total_angle(uint8_t idx)//复用更新多圈角度
{
  if (motor_info[idx].rotor_angle == 0) return;
  angle_update_total(&total_angle[idx], motor_info[idx].rotor_angle, &last_enc[idx]);
}


static void set_two_motor_output(int16_t out0, int16_t out1)//双电机输出
{
  set_motor_voltage(0, out0, out1, 0, 0);
}


static void boot_calibration_if_needed(void)//上电校准主函数
{
    /* 需要 rc 数据，所以确保 dbus_uart_init() 已经调用且有时间收到一帧 */
    HAL_Delay(200);

    /* 约定：SW2==1 进入校准模式 */
    if (rc.sw2 != 1) return;

    /* 等 CAN 反馈有效 */
    while (motor_info[MOTOR0_IDX].rotor_angle == 0 || motor_info[MOTOR1_IDX].rotor_angle == 0) {
        HAL_Delay(1);
    }

    /* 初始化多圈角 */
    last_enc[MOTOR0_IDX] = motor_info[MOTOR0_IDX].rotor_angle;
    last_enc[MOTOR1_IDX] = motor_info[MOTOR1_IDX].rotor_angle;
    total_angle[MOTOR0_IDX] = 0;
    total_angle[MOTOR1_IDX] = 0;
    update_total_angle(MOTOR0_IDX);
    update_total_angle(MOTOR1_IDX);

    /* 校准时输出比例（你可以按手感调小点更安全） */
    const int16_t DEAD_ZONE = 20;
    const float   OPEN_LOOP_SCALE = 10.0f;   // 摇杆到输出的比例
    const int16_t OUT_MAX = 8000;            // 开环输出限幅

    last_sw2 = rc.sw2;

    /* 校准循环：SW2拨回2退出 */
    while (1) {
        update_total_angle(MOTOR0_IDX);
        update_total_angle(MOTOR1_IDX);

        /* 开环跟随摇杆：CH2->电机0, CH3->电机1 */
        int16_t out0 = 0, out1 = 0;
        if (abs(rc.ch2) > DEAD_ZONE) out0 = (int16_t)(rc.ch2 * OPEN_LOOP_SCALE);
        if (abs(rc.ch3) > DEAD_ZONE) out1 = (int16_t)(rc.ch3 * OPEN_LOOP_SCALE);

        /* 限幅 */
        if (out0 > OUT_MAX) out0 = OUT_MAX;
        if (out0 < -OUT_MAX) out0 = -OUT_MAX;
        if (out1 > OUT_MAX) out1 = OUT_MAX;
        if (out1 < -OUT_MAX) out1 = -OUT_MAX;

        set_two_motor_output(out0, out1);

        /* SW2边沿：从非1变成1 => 记录下限；从非3变成3 => 记录上限 */
        if (rc.sw2 != last_sw2) {
            if (rc.sw2 == 1) {
                mech_min[MOTOR0_IDX] = total_angle[MOTOR0_IDX];
                mech_min[MOTOR1_IDX] = total_angle[MOTOR1_IDX];
            } else if (rc.sw2 == 3) {
                mech_max[MOTOR0_IDX] = total_angle[MOTOR0_IDX];
                mech_max[MOTOR1_IDX] = total_angle[MOTOR1_IDX];
            }
            last_sw2 = rc.sw2;
        }

        /* 退出条件：SW2==2 */
        if (rc.sw2 == 2) {
            break;
        }

        HAL_Delay(10);
    }

    /* 退出前停输出 */
    set_two_motor_output(0, 0);

    /* 保护：如果记录反了就交换 */
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (mech_min[i] > mech_max[i]) {
            int32_t tmp = mech_min[i];
            mech_min[i] = mech_max[i];
            mech_max[i] = tmp;
        }
    }
  g_calibrated = true;

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
    pwm_init();                              // start pwm output//没啥用估计是通用的
    can_user_init(&hcan1);                   // config can filter, start can

    boot_calibration_if_needed();/* 上电校准（SW2==1 进入） */
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
      //sprintf(buf, "CH1: %4d  CH2: %4d  CH3: %4d  CH4: %4d  SW1: %1d  SW2: %1d \n", rc.ch1, rc.ch2, rc.ch3, rc.ch4, rc.sw1, rc.sw2);
      //HAL_UART_Transmit(&huart6, (uint8_t *)buf, COUNTOF(buf) , 55);
      HAL_Delay(1);
      position_control_loop();
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
