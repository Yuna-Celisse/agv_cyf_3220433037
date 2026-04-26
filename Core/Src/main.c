/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rc522.h"
#include "oled.h"
#include "app_encoder.h"
#include "app_line.h"
#include "app_motor.h"
#include "app_servo.h"
#include "app_oled_ui.h"
#include "app_ultrasonic.h"
#include "app_rfid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENABLE_RFID_UART_REPORT 0U                /* RFID UID 是否通过串口打印：0=关闭，1=开启 */
#define ENABLE_ULTRASONIC_UART_REPORT 1U          /* 超声波测距结果是否通过串口打印 */
#define CARD_STANDBY_DELAY_MS 2000U               /* 刷卡后静止等待时间（ms），用于给系统稳定时间 */
#define HCSR04_MEASURE_INTERVAL_MS 100U           /* 静止/非运动状态下超声波触发周期（ms） */
#define HCSR04_MOVE_INTERVAL_MS 140U              /* 运动状态下超声波触发周期（ms） */
#define HCSR04_MAX_JUMP_CM 25U                    /* 两次距离读数允许的最大跳变（cm） */
#define HCSR04_INVALID_STOP_COUNT 3U              /* 连续无效测距达到该次数后触发停机保护 */
#define RESTORE_STAGE_US_ONLY 0U                  /* 恢复阶段：仅超声波 */
#define RESTORE_STAGE_SENSOR_UI 1U                /* 恢复阶段：传感器 + UI */
#define RESTORE_STAGE_FULL 2U                     /* 恢复阶段：完整功能 */
#define SYSTEM_RESTORE_STAGE RESTORE_STAGE_FULL   /* 当前系统启用的恢复阶段 */
#define ENABLE_ULTRASONIC_ONLY ((SYSTEM_RESTORE_STAGE) == RESTORE_STAGE_US_ONLY)          /* 是否仅启用超声波功能 */
#define ENABLE_NON_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_SENSOR_UI)    /* 是否启用非运动类功能（OLED/RFID等） */
#define ENABLE_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)              /* 是否启用运动控制功能 */
#define ENABLE_RFID_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)                /* 是否启用 RFID 功能 */
#define ENABLE_MOTOR_CLOSED_LOOP 0U                /* 电机闭环控制开关：0=开环，1=闭环 */
/* Duty values are in per-mille: 0..1000 */
#define LINE_BASE_DUTY_PM 160U                    /* 巡线基础占空比（千分比） */
#define LINE_MIN_DUTY_PM 50U                      /* 巡线最小占空比（千分比） */
#define LINE_MAX_DUTY_PM 650U                     /* 巡线最大占空比（千分比） */
#define LINE_PID_INTERVAL_MS 10U                  /* 巡线控制周期（ms） */
/* Integer proportional gain: correction_pm = error_x10 * LINE_KP_PM_PER_ERR10 */
#define LINE_KP_PM_PER_ERR10 8                    /* 比例系数Kp（整型）：误差每增加10单位时的占空比修正 */
#define SERVO_RUN_ANGLE_DEG 55U                   /* 正常运行时舵机角度（度） */
#define SERVO_DEFAULT_ANGLE_DEG SERVO_RUN_ANGLE_DEG /* 舵机上电默认角度 */
#define SERVO_ENABLE 1U                           /* 舵机功能开关（预留） */
#define SERVO_ROTATE_DURATION_MS 2000U            /* 停车流程中舵机旋转阶段时长（ms） */
#define SERVO_HOLD_DURATION_MS 2000U              /* 停车流程中舵机保持阶段时长（ms） */
#define SERVO_STOP_START_ANGLE_DEG SERVO_RUN_ANGLE_DEG /* 停车流程旋转起始角 */
#define SERVO_STOP_END_ANGLE_DEG 90U              /* 停车流程旋转终止角 */
#define LINE_BLACK_CONFIRM_FRAMES 3U              /* 识别“经过一条黑线”所需的连续确认帧数 */
#define OBSTACLE_THRESHOLD_CM 20U                 /* 障碍物阈值距离（cm），小于等于该值进入避障 */
#define AVOID_RIGHT_MS 620U                       /* 右转搜索阶段预设时长（ms） */
#define AVOID_LEFT_MS 850U                        /* 左转绕障阶段时长（ms） */
#define AVOID_FORWARD_MS 180U                     /* 前进绕障阶段时长（ms） */
#define AVOID_RIGHT_ALIGN_MS 520U                 /* 右转对齐阶段时长（ms） */
#define AVOID_RETURN_FORWARD_MS 1400U             /* 回到线路方向前进阶段时长（ms） */
#define AVOID_RIGHT_SEARCH_MAX_MS 1600U           /* 右转找线阶段最大允许时长（ms） */
#define AVOID_LINE_CONFIRM_FRAMES 2U              /* 避障找线时识别到线的连续确认帧数 */
#define AVOID_DUTY_FAST_PM 220U                   /* 避障快速轮占空比（预留） */
#define AVOID_DUTY_SLOW_PM 80U                    /* 避障慢速轮占空比（预留） */
#define AVOID_RIGHT_DUTY_FAST_PM 260U             /* 右转时快侧轮占空比 */
#define AVOID_RIGHT_DUTY_SLOW_PM 50U              /* 右转时慢侧轮占空比 */
#define AVOID_LEFT_DUTY_FAST_PM 260U              /* 左转时快侧轮占空比 */
#define AVOID_LEFT_DUTY_SLOW_PM 50U               /* 左转时慢侧轮占空比 */
#define AVOID_FORWARD_DUTY_PM 230U                /* 避障前进占空比 */
#define AVOID_RETURN_FORWARD_DUTY_PM 230U         /* 回位前进占空比 */
#define AVOID_RIGHT_ALIGN_DUTY_FAST_PM 260U       /* 右转对齐时快侧轮占空比 */
#define AVOID_RIGHT_ALIGN_DUTY_SLOW_PM 80U        /* 右转对齐时慢侧轮占空比 */

#define CARD_ID_NONE 0U                           /* 未识别到有效任务卡 */
#define CARD_ID_A 1U                              /* A卡：对应第1条线停车 */
#define CARD_ID_B 2U                              /* B卡：对应第2条线停车 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t last_ultrasonic_poll_tick = 0U;   /* 上一次触发超声波测距的时间戳 */
static uint16_t last_distance_cm = 0U;            /* 最近一次有效距离值（cm） */
static uint8_t has_last_distance = 0U;            /* 是否已有有效距离缓存 */
static uint8_t ultrasonic_jump_reject_count = 0U; /* 大跳变抑制计数器 */
static uint8_t ultrasonic_invalid_count = 0U;     /* 连续无效测距计数器 */
static uint32_t last_line_pid_tick = 0U;          /* 上一次执行巡线控制的时间戳 */

typedef enum
{
  VEHICLE_WAIT_CARD = 0,
  VEHICLE_CARD_STANDBY,
  VEHICLE_LINE_FOLLOW,
  VEHICLE_STOPPING,
  VEHICLE_AVOID_RIGHT,
  VEHICLE_AVOID_LEFT,
  VEHICLE_AVOID_FORWARD,
  VEHICLE_AVOID_RIGHT_ALIGN,
  VEHICLE_AVOID_RETURN_FORWARD
} VehicleState_t;

static VehicleState_t vehicle_state = VEHICLE_WAIT_CARD; /* 当前车辆状态机状态 */
static uint8_t target_card = CARD_ID_NONE;               /* 当前任务卡类型 */
static uint8_t target_stop_line = 0U;                    /* 目标停车线编号 */
static uint8_t crossed_line_count = 0U;                  /* 已经过黑线计数 */
static uint8_t line_black_frame_count = 0U;              /* 当前黑线连续帧计数 */
static uint8_t line_black_seen_mask = 0U;                /* 当前黑线连续帧中出现过的传感器位图 */
static uint8_t line_cross_latched = 0U;                  /* 过线锁存标志，避免重复计数 */
static uint8_t avoid_line_seen_count = 0U;               /* 避障阶段重新找线确认计数 */
static uint32_t state_start_tick = 0U;                   /* 当前状态进入时刻 */
static uint32_t stop_start_tick = 0U;                    /* 停车流程开始时刻 */
static uint8_t stop_at_next_line = 0U;                   /* 是否将在下一条线最终停车 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void HCSR04_InitPins(void);
static void StartMission(uint8_t card_id);
static uint8_t IsUltrasonicMotionState(VehicleState_t state);
static void ResetUltrasonicState(void);
static void AppSetAvoidTurn(uint8_t turn_left, uint16_t fast_pm, uint16_t slow_pm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void HCSR04_InitPins(void)
{
  /* 初始化超声波模块（包含是否串口打印的配置）。 */
  AppUltrasonic_Init(ENABLE_ULTRASONIC_UART_REPORT);
}

static uint8_t IsUltrasonicMotionState(VehicleState_t state)
{
  /* 仅在以下运动相关状态中，才将超声波视为“运动模式测距”。 */
  return (uint8_t)((state == VEHICLE_LINE_FOLLOW) ||
                   (state == VEHICLE_AVOID_RIGHT) ||
                   (state == VEHICLE_AVOID_LEFT) ||
                   (state == VEHICLE_AVOID_FORWARD) ||
                   (state == VEHICLE_AVOID_RIGHT_ALIGN) ||
                   (state == VEHICLE_AVOID_RETURN_FORWARD));
}

static void ResetUltrasonicState(void)
{
  /* 清空距离缓存与异常计数，通常在任务启动或状态重置时调用。 */
  last_distance_cm = 0U;
  has_last_distance = 0U;
  ultrasonic_jump_reject_count = 0U;
  ultrasonic_invalid_count = 0U;
}

static void AppSetAvoidTurn(uint8_t turn_left, uint16_t fast_pm, uint16_t slow_pm)
{
  /* 通过左右轮占空比差实现原地/小半径转向：左转时左慢右快，右转时左快右慢。 */
  if (turn_left != 0U)
  {
    AppMotor_SetTargetFromDuty(fast_pm, slow_pm);
  }
  else
  {
    AppMotor_SetTargetFromDuty(slow_pm, fast_pm);
  }
}

static void StartMission(uint8_t card_id)
{
  /* 仅接受 A/B 两种任务卡；其他值直接忽略。 */
  if ((card_id != CARD_ID_A) && (card_id != CARD_ID_B))
  {
    return;
  }

  /* 根据卡片确定任务目标：A 停第1条线，B 停第2条线。 */
  target_card = card_id;
  target_stop_line = (card_id == CARD_ID_A) ? 1U : 2U;
  /* 清空全部过程状态，确保每次任务都从一致起点开始。 */
  crossed_line_count = 0U;
  line_black_frame_count = 0U;
  line_black_seen_mask = 0U;
  line_cross_latched = 0U;
  avoid_line_seen_count = 0U;
  last_line_pid_tick = 0U;
  stop_at_next_line = 0U;
  ResetUltrasonicState();
  /* 进入刷卡后待发车状态，并记录状态起始时间。 */
  vehicle_state = VEHICLE_CARD_STANDBY;
  state_start_tick = HAL_GetTick();
  /* 电机先禁能并归零占空比，舵机回到运行位。 */
  AppMotor_SetForwardDirection();
  AppMotor_SetEnable(0U);
  AppMotor_SetDuty(0U, 0U);
  /* OLED 显示当前任务目标。 */
  AppServo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
  AppOled_ShowTarget(card_id);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); /* HAL 库初始化：复位外设、初始化 SysTick 等基础资源。 */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); /* 配置系统主频和总线时钟。 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();         /* GPIO 初始化。 */
  MX_TIM1_Init();         /* TIM1：PWM（电机驱动）相关。 */
  MX_TIM2_Init();         /* TIM2：微秒计时/测距时基相关。 */
  MX_TIM3_Init();         /* TIM3：项目内其他定时功能。 */
  MX_TIM4_Init();         /* TIM4：编码器/控制相关定时。 */
  MX_USART3_UART_Init();  /* 串口3初始化（调试输出/模块通信）。 */
  /* USER CODE BEGIN 2 */
  AppEncoder_Init(); /* 编码器模块初始化。 */
#if ENABLE_NON_MOTION_FEATURES
  OLED_Init();                /* OLED 屏初始化。 */
  AppOled_ShowSwipePrompt();  /* 提示“请刷卡”。 */
#if ENABLE_RFID_FEATURES
  AppRfid_Init();             /* RFID 模块初始化。 */
#endif
#endif
#if ENABLE_MOTION_FEATURES
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     /* 启动左轮PWM通道。 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);     /* 启动右轮PWM通道。 */
  AppMotor_SetClosedLoop(ENABLE_MOTOR_CLOSED_LOOP); /* 配置电机开/闭环模式。 */
  AppMotor_SetForwardDirection();               /* 默认前进方向。 */
  AppMotor_SetEnable(0U);                       /* 上电后先禁能，防止误动作。 */
#endif
  HCSR04_InitPins(); /* 超声波模块初始化。 */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0U); /* 清零 TIM2 计数器。 */
#if ENABLE_MOTION_FEATURES
  AppServo_Init(SERVO_DEFAULT_ANGLE_DEG); /* 舵机初始化到默认角度。 */
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t ir_mask = AppLine_ReadMask(); /* 读取巡线红外原始位图。 */
    uint32_t now = HAL_GetTick();         /* 获取当前毫秒时刻。 */

#if ENABLE_MOTION_FEATURES
  AppServo_Task(); /* 舵机周期任务，保证平滑控制。 */
#endif

#if ENABLE_NON_MOTION_FEATURES
#if ENABLE_RFID_FEATURES
    {
      AppRfidEvent_t rfid_event;
      uint8_t allow_rfid_poll = (uint8_t)((AppUltrasonic_IsBusy() == 0U) &&
                                          (vehicle_state == VEHICLE_WAIT_CARD) &&
                                          (target_card == CARD_ID_NONE));
      /* 仅在超声波空闲 + 等待刷卡 + 未锁定任务时轮询 RFID，减少冲突。 */

      if (AppRfid_Poll(now, allow_rfid_poll, &rfid_event) != 0U)
      {
        if (rfid_event.is_new_uid != 0U)
        {
          uint8_t card_id;

          AppOled_ShowUid(rfid_event.uid, ENABLE_RFID_UART_REPORT); /* OLED/串口显示刷到的UID。 */
          card_id = rfid_event.card_id;                             /* 取解析后的任务卡类型。 */

          if ((vehicle_state == VEHICLE_WAIT_CARD) && ((card_id == CARD_ID_A) || (card_id == CARD_ID_B)))
          {
            StartMission(card_id); /* 启动任务流程。 */
          }
        }
      }
    }
#endif
#endif

    uint32_t ultrasonic_interval_ms = HCSR04_MEASURE_INTERVAL_MS; /* 默认测距周期（静止模式）。 */
    uint8_t ultrasonic_enabled = 1U;                              /* 默认使能超声波。 */

  #if ENABLE_MOTION_FEATURES
    if (vehicle_state == VEHICLE_WAIT_CARD)
    {
      ultrasonic_enabled = 0U; /* 等待刷卡时可关闭主动测距，降低干扰。 */
    }

    if (IsUltrasonicMotionState(vehicle_state) != 0U)
    {
      ultrasonic_interval_ms = HCSR04_MOVE_INTERVAL_MS; /* 运动时改用更稳妥的测距间隔。 */
    }
  #endif

    AppUltrasonic_Task(now); /* 维护超声波状态机。 */

    {
      uint16_t distance_cm;      /* 本次取出的距离值。 */
      uint8_t has_distance = 0U; /* 本次结果是否有效。 */

      if (AppUltrasonic_FetchResult(&distance_cm, &has_distance) != 0U)
      {
        if (has_distance != 0U)
        {
#if ENABLE_MOTION_FEATURES
          if ((has_last_distance != 0U) &&
              (IsUltrasonicMotionState(vehicle_state) != 0U))
          {
            /* 对运动状态下的距离突变做抑制，过滤偶发噪声点。 */
            uint16_t diff_cm = (distance_cm > last_distance_cm) ?
              (uint16_t)(distance_cm - last_distance_cm) :
              (uint16_t)(last_distance_cm - distance_cm);

            if (diff_cm > HCSR04_MAX_JUMP_CM)
            {
              if ((distance_cm < last_distance_cm) && (distance_cm <= OBSTACLE_THRESHOLD_CM))
              {
                ultrasonic_jump_reject_count = 0U; /* 若突变向近距离且疑似真实障碍，允许立即生效。 */
              }
              else if (ultrasonic_jump_reject_count < 2U)
              {
                ultrasonic_jump_reject_count++;
                distance_cm = last_distance_cm; /* 先沿用上一帧，等待确认。 */
              }
              else
              {
                ultrasonic_jump_reject_count = 0U; /* 连续多次后放行，避免永久卡死。 */
              }
            }
            else
            {
              ultrasonic_jump_reject_count = 0U; /* 无突变则清零抑制计数。 */
            }
          }
          else
          {
            ultrasonic_jump_reject_count = 0U; /* 非运动态/首次数据不做突变判断。 */
          }
#endif

          last_distance_cm = distance_cm; /* 更新有效距离缓存。 */
          has_last_distance = 1U;         /* 标记已有有效数据。 */
          ultrasonic_invalid_count = 0U;  /* 有效数据到来，清空无效计数。 */
        }
        else
        {
          has_last_distance = 0U; /* 本次无效，清掉有效标记。 */
#if ENABLE_MOTION_FEATURES
          if (IsUltrasonicMotionState(vehicle_state) != 0U)
          {
            if (ultrasonic_invalid_count < 255U)
            {
              ultrasonic_invalid_count++; /* 运动中无效读数累计，用于安全停机。 */
            }
          }
          else
          {
            ultrasonic_invalid_count = 0U; /* 非运动状态不累计。 */
          }
#else
          ultrasonic_invalid_count = 0U;
#endif
        }
      }
    }

    if ((ultrasonic_enabled != 0U) && ((now - last_ultrasonic_poll_tick) >= ultrasonic_interval_ms))
    {
      last_ultrasonic_poll_tick = now; /* 更新触发时间戳。 */
      AppUltrasonic_StartMeasure();    /* 发起一次新测距。 */
    }

#if !ENABLE_MOTION_FEATURES
  #if ENABLE_NON_MOTION_FEATURES
    AppOled_ShowDistance(has_last_distance, last_distance_cm); /* 仅显示距离，不进入运动状态机。 */
  #endif
    continue;
#endif

    if ((IsUltrasonicMotionState(vehicle_state) != 0U) &&
        (ultrasonic_invalid_count >= HCSR04_INVALID_STOP_COUNT))
    {
      AppMotor_SetEnable(0U);  /* 连续无效测距时立即禁能电机。 */
      AppMotor_SetDuty(0U, 0U);/* 输出占空比归零。 */
      if (vehicle_state == VEHICLE_LINE_FOLLOW)
      {
        last_line_pid_tick = now; /* 保持巡线时基连续。 */
      }
      else
      {
        state_start_tick = now;   /* 其他状态重置阶段计时。 */
      }
      AppOled_ShowDistance(has_last_distance, last_distance_cm);
      continue;
    }

    switch (vehicle_state)
    {
      case VEHICLE_WAIT_CARD:
        AppMotor_SetEnable(0U);                /* 等待刷卡：电机禁能。 */
        AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);/* 舵机保持运行位。 */
        AppOled_ShowSwipePrompt();             /* OLED 提示刷卡。 */
        break;

      case VEHICLE_CARD_STANDBY:
        AppMotor_SetEnable(0U);                /* 待发车阶段电机关闭。 */
        AppMotor_SetDuty(0U, 0U);              /* 再次确保占空比为0。 */
        AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);/* 舵机保持运行位。 */
        if ((now - state_start_tick) >= CARD_STANDBY_DELAY_MS)
        {
          vehicle_state = VEHICLE_LINE_FOLLOW; /* 等待时间到，进入巡线。 */
        }
        break;

      case VEHICLE_LINE_FOLLOW:
      {
        uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);

        AppServo_SetAngle(SERVO_RUN_ANGLE_DEG); /* 巡线中舵机保持运行角。 */

        if ((has_last_distance != 0U) && (last_distance_cm <= OBSTACLE_THRESHOLD_CM))
        {
          vehicle_state = VEHICLE_AVOID_LEFT; /* 探测到障碍，进入避障序列。 */
          avoid_line_seen_count = 0U;         /* 清空找线计数。 */
          state_start_tick = now;             /* 记录避障状态起点时间。 */
          break;
        }

        if ((now - last_line_pid_tick) >= LINE_PID_INTERVAL_MS)
        {
          int16_t error_x10 = 0;
          int16_t correction_pm;
          int32_t duty_left_pm;
          int32_t duty_right_pm;

          last_line_pid_tick = now; /* 更新巡线控制节拍。 */

          if (AppLine_ComputeErrorX10(ir_mask, &error_x10) == 0U)
          {
            error_x10 = 0; /* 计算失败时按居中处理，避免异常输出。 */
          }

          correction_pm = (int16_t)(error_x10 * LINE_KP_PM_PER_ERR10);
          duty_left_pm = (int32_t)LINE_BASE_DUTY_PM - correction_pm;
          duty_right_pm = (int32_t)LINE_BASE_DUTY_PM + correction_pm;

          if (duty_left_pm < (int32_t)LINE_MIN_DUTY_PM)
          {
            duty_left_pm = LINE_MIN_DUTY_PM;
          }
          else if (duty_left_pm > (int32_t)LINE_MAX_DUTY_PM)
          {
            duty_left_pm = LINE_MAX_DUTY_PM;
          }

          if (duty_right_pm < (int32_t)LINE_MIN_DUTY_PM)
          {
            duty_right_pm = LINE_MIN_DUTY_PM;
          }
          else if (duty_right_pm > (int32_t)LINE_MAX_DUTY_PM)
          {
            duty_right_pm = LINE_MAX_DUTY_PM;
          }

          AppMotor_SetEnable(1U); /* 巡线时使能电机。 */
          AppMotor_SetTargetFromDuty((uint16_t)duty_left_pm, (uint16_t)duty_right_pm); /* 输出左右轮目标占空比。 */

          if (line_cross_latched == 0U)
          {
            if (norm_mask != 0U)
            {
              if (line_black_frame_count < 255U)
              {
                line_black_frame_count++;
              }
              line_black_seen_mask |= norm_mask;

              if ((line_black_frame_count >= LINE_BLACK_CONFIRM_FRAMES) && (line_black_seen_mask == 0x1FU))
              {
                crossed_line_count++;
                line_cross_latched = 1U;
                line_black_frame_count = 0U;
                line_black_seen_mask = 0U;
                /* 识别到一次完整过线事件并锁存，避免一条线重复计数。 */

                if ((target_stop_line != 0U) && (crossed_line_count >= target_stop_line))
                {
                  vehicle_state = VEHICLE_STOPPING;
                  stop_start_tick = now;
                  AppMotor_SetEnable(0U);
                  AppMotor_SetDuty(0U, 0U);
                  /* 到达目标线后进入停车流程，先立即停电机。 */

                  if (stop_at_next_line != 0U)
                  {
                    target_stop_line = 0U;
                  }
                }
              }
            }
            else
            {
              line_black_frame_count = 0U;
              line_black_seen_mask = 0U;
            }
          }
          else if (norm_mask != 0x1FU)
          {
            line_cross_latched = 0U;
            line_black_frame_count = 0U;
            line_black_seen_mask = 0U;
          }
        }
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        break;
      }

      case VEHICLE_STOPPING:
      {
        uint32_t elapsed = now - stop_start_tick;
        uint16_t angle;

        AppMotor_SetEnable(0U);
        AppMotor_SetDuty(0U, 0U);

        if (stop_at_next_line != 0U)
        {
          /* 第二次到站：复位任务并回到等待刷卡状态。 */
          AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
          vehicle_state = VEHICLE_WAIT_CARD;
          target_card = CARD_ID_NONE;
          target_stop_line = 0U;
          crossed_line_count = 0U;
          line_black_frame_count = 0U;
          line_black_seen_mask = 0U;
          line_cross_latched = 0U;
          stop_at_next_line = 0U;
        }
        else if (elapsed >= (SERVO_ROTATE_DURATION_MS + SERVO_HOLD_DURATION_MS))
        {
          /* 首次停车流程结束：重新起步，目标改为“下一条线停车”。 */
          AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
          stop_at_next_line = 1U;
          target_stop_line = (uint8_t)(crossed_line_count + 1U);
          line_black_frame_count = 0U;
          line_black_seen_mask = 0U;
          line_cross_latched = 0U;
          last_line_pid_tick = now;
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        else if (elapsed >= SERVO_ROTATE_DURATION_MS)
        {
          AppServo_SetAngle(SERVO_STOP_END_ANGLE_DEG); /* 旋转完成后保持终点角。 */
        }
        else
        {
          /* 旋转阶段：按时间线性插值，实现平滑转动。 */
          int32_t angle_delta = (int32_t)SERVO_STOP_END_ANGLE_DEG - (int32_t)SERVO_STOP_START_ANGLE_DEG;
          angle = (int16_t)((int32_t)SERVO_STOP_START_ANGLE_DEG +
            ((angle_delta * (int32_t)elapsed) / (int32_t)SERVO_ROTATE_DURATION_MS));
          AppServo_SetAngle((uint16_t)angle);
        }

        break;
      }

      case VEHICLE_AVOID_RIGHT:
      {
        uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);
        uint32_t elapsed = now - state_start_tick;

        AppMotor_SetEnable(1U); /* 避障找线阶段保持电机使能。 */
        AppSetAvoidTurn(0U, AVOID_RIGHT_DUTY_FAST_PM, AVOID_RIGHT_DUTY_SLOW_PM); /* 右转找线。 */
        AppOled_ShowDistance(has_last_distance, last_distance_cm); /* 实时显示距离。 */

        if (norm_mask != 0U)
        {
          if (avoid_line_seen_count < 255U)
          {
            avoid_line_seen_count++;
          }
        }
        else
        {
          avoid_line_seen_count = 0U;
        }

        if ((avoid_line_seen_count >= AVOID_LINE_CONFIRM_FRAMES) ||
            (elapsed >= AVOID_RIGHT_SEARCH_MAX_MS))
        {
          /* 找到线或超时都回到巡线状态。 */
          avoid_line_seen_count = 0U;
          line_black_frame_count = 0U;
          line_black_seen_mask = 0U;
          line_cross_latched = 0U;
          last_line_pid_tick = now;
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        break;
      }

      case VEHICLE_AVOID_LEFT:
        AppMotor_SetEnable(1U); /* 左转绕开障碍。 */
        AppSetAvoidTurn(1U, AVOID_LEFT_DUTY_FAST_PM, AVOID_LEFT_DUTY_SLOW_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_LEFT_MS)
        {
          vehicle_state = VEHICLE_AVOID_FORWARD;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_FORWARD:
        AppMotor_SetEnable(1U); /* 直行绕障。 */
        AppMotor_SetTargetFromDuty(AVOID_FORWARD_DUTY_PM, AVOID_FORWARD_DUTY_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_FORWARD_MS)
        {
          vehicle_state = VEHICLE_AVOID_RIGHT_ALIGN;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_RIGHT_ALIGN:
        AppMotor_SetEnable(1U); /* 右转对齐回线路方向。 */
        AppSetAvoidTurn(0U, AVOID_RIGHT_ALIGN_DUTY_FAST_PM, AVOID_RIGHT_ALIGN_DUTY_SLOW_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_RIGHT_ALIGN_MS)
        {
          vehicle_state = VEHICLE_AVOID_RETURN_FORWARD;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_RETURN_FORWARD:
        AppMotor_SetEnable(1U); /* 向前回位，准备重新找线。 */
        AppMotor_SetTargetFromDuty(AVOID_RETURN_FORWARD_DUTY_PM, AVOID_RETURN_FORWARD_DUTY_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_RETURN_FORWARD_MS)
        {
          vehicle_state = VEHICLE_AVOID_RIGHT;
          avoid_line_seen_count = 0U;
          state_start_tick = now;
        }
        break;

      default:
        vehicle_state = VEHICLE_WAIT_CARD; /* 兜底保护：异常状态回到等待刷卡。 */
        break;
    }

    AppMotor_Task(now); /* 执行电机控制周期任务（闭环/输出更新等）。 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  AppUltrasonic_HandleEchoExti(GPIO_Pin); /* 将外部中断引脚交给超声波模块处理回波。 */
  AppEncoder_HandleExti(GPIO_Pin);        /* 将外部中断引脚交给编码器模块计数。 */
}

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
    /* 错误态死循环：等待调试器介入。 */
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
