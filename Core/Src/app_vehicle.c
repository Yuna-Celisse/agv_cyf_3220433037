#include "app_vehicle.h"

#include "app_encoder.h"
#include "app_line.h"
#include "app_motor.h"
#include "app_oled_ui.h"
#include "app_rfid.h"
#include "app_servo.h"
#include "app_ultrasonic.h"
#include "main.h"
#include "oled.h"
#include "tim.h"
#include "usart.h"

#include <stdio.h>

/*
 * 这是整车的核心调度模块。
 * 它把 RFID、循迹、超声波避障、舵机取放、OLED 显示、电机驱动
 * 这些能力统一组织成一个有限状态机，让主循环只需要调用
 * AppVehicle_Task() 就能驱动整车完整运行。
 */

#define ENABLE_RFID_UART_REPORT 0U
#define ENABLE_ULTRASONIC_UART_REPORT 1U
#define CARD_STANDBY_DELAY_MS 2000U
#define HCSR04_MEASURE_INTERVAL_MS 100U
#define HCSR04_MOVE_INTERVAL_MS 140U
#define HCSR04_MAX_JUMP_CM 25U
#define HCSR04_INVALID_STOP_COUNT 3U
#define RESTORE_STAGE_US_ONLY 0U
#define RESTORE_STAGE_SENSOR_UI 1U
#define RESTORE_STAGE_FULL 2U
#define SYSTEM_RESTORE_STAGE RESTORE_STAGE_FULL
#define ENABLE_ULTRASONIC_ONLY ((SYSTEM_RESTORE_STAGE) == RESTORE_STAGE_US_ONLY)
#define ENABLE_NON_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_SENSOR_UI)
#define ENABLE_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)
#define ENABLE_RFID_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)
#define ENABLE_MOTOR_CLOSED_LOOP 0U
#define ENABLE_ENCODER_UART_REPORT 1U
#define ENCODER_UART_REPORT_INTERVAL_MS 100U
#define LINE_BASE_DUTY_PM 160U
#define LINE_MIN_DUTY_PM 50U
#define LINE_MAX_DUTY_PM 650U
#define LINE_PID_INTERVAL_MS 10U
#define LINE_KP_PM_PER_ERR10 8
#define SERVO_RUN_ANGLE_DEG 55U
#define SERVO_DEFAULT_ANGLE_DEG SERVO_RUN_ANGLE_DEG
#define SERVO_HOLD_DURATION_MS 2000U
#define SERVO_STOP_START_ANGLE_DEG SERVO_RUN_ANGLE_DEG
#define SERVO_STOP_END_ANGLE_DEG 90U
#define LINE_BLACK_CONFIRM_FRAMES 3U
#define OBSTACLE_THRESHOLD_CM 20U
#define AVOID_LEFT_MS 850U
#define AVOID_FORWARD_MS 180U
#define AVOID_RIGHT_ALIGN_MS 760U
#define AVOID_RETURN_FORWARD_MS 1400U
#define AVOID_RIGHT_SEARCH_MAX_MS 1600U
#define AVOID_LINE_CONFIRM_FRAMES 1U
#define AVOID_LINE_EDGE_MASK 0x11U
#define LINE_RECOVER_BOOST_MS 220U
#define LINE_RECOVER_KP_NUM 1
#define AVOID_RIGHT_DUTY_FAST_PM 220U
#define AVOID_RIGHT_DUTY_SLOW_PM 70U
#define AVOID_LEFT_DUTY_FAST_PM 260U
#define AVOID_LEFT_DUTY_SLOW_PM 50U
#define AVOID_FORWARD_DUTY_PM 230U
#define AVOID_RETURN_FORWARD_DUTY_PM 230U
#define AVOID_RIGHT_ALIGN_DUTY_FAST_PM 270U
#define AVOID_RIGHT_ALIGN_DUTY_SLOW_PM 70U

#define CARD_ID_NONE 0U
#define CARD_ID_A 1U
#define CARD_ID_B 2U

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

static uint32_t s_last_ultrasonic_poll_tick = 0U;
static uint16_t s_last_distance_cm = 0U;
static uint8_t s_has_last_distance = 0U;
static uint8_t s_ultrasonic_jump_reject_count = 0U;
static uint8_t s_ultrasonic_invalid_count = 0U;
static uint32_t s_last_line_pid_tick = 0U;
static VehicleState_t s_vehicle_state = VEHICLE_WAIT_CARD;
static uint8_t s_target_card = CARD_ID_NONE;
static uint8_t s_target_stop_line = 0U;
static uint8_t s_final_stop_line = 0U;
static uint8_t s_crossed_line_count = 0U;
static uint8_t s_line_black_frame_count = 0U;
static uint8_t s_line_black_seen_mask = 0U;
static uint8_t s_line_cross_latched = 0U;
static uint8_t s_avoid_line_seen_count = 0U;
static uint8_t s_oled_showing_avoid = 0U;
static uint8_t s_pickup_done = 0U;
static uint8_t s_payload_raised = 0U;
static uint8_t s_stop_is_final = 0U;
static uint32_t s_line_recover_until_tick = 0U;
static uint32_t s_state_start_tick = 0U;
static uint32_t s_stop_start_tick = 0U;
static uint32_t s_last_encoder_report_tick = 0U;

/*
 * 只有在车辆真正运动时，超声波连续异常才应该被视为故障。
 * 如果当前只是待机、刷卡或停止状态，传感器偶尔无效不需要升级成故障。
 */
static uint8_t AppVehicle_IsUltrasonicMotionState(VehicleState_t state)
{
  return (uint8_t)((state == VEHICLE_LINE_FOLLOW) ||
                   (state == VEHICLE_AVOID_RIGHT) ||
                   (state == VEHICLE_AVOID_LEFT) ||
                   (state == VEHICLE_AVOID_FORWARD) ||
                   (state == VEHICLE_AVOID_RIGHT_ALIGN) ||
                   (state == VEHICLE_AVOID_RETURN_FORWARD));
}

/* 连续多次拿不到有效距离时，认为超声波当前不可靠。 */
static uint8_t AppVehicle_IsUltrasonicFaulted(void)
{
  return (uint8_t)(s_ultrasonic_invalid_count >= HCSR04_INVALID_STOP_COUNT);
}

/* 清空超声波历史距离、突变过滤计数和无效结果计数。 */
static void AppVehicle_ResetUltrasonicState(void)
{
  s_last_distance_cm = 0U;
  s_has_last_distance = 0U;
  s_ultrasonic_jump_reject_count = 0U;
  s_ultrasonic_invalid_count = 0U;
}

/*
 * 清空循迹相关状态。
 * 包括已经过了几条线、是否处于整条黑线锁存状态、避障后回线计数等。
 */
static void AppVehicle_ResetLineTracking(void)
{
  s_crossed_line_count = 0U;
  s_line_black_frame_count = 0U;
  s_line_black_seen_mask = 0U;
  s_line_cross_latched = 0U;
  s_avoid_line_seen_count = 0U;
  s_last_line_pid_tick = 0U;
  s_stop_is_final = 0U;
}

/* 统一的停车动作，保证所有停止路径都走相同的底层控制。 */
static void AppVehicle_StopMotion(void)
{
  AppMotor_SetEnable(0U);
  AppMotor_SetDuty(0U, 0U);
}

/*
 * 避障回到主线后，给一个短暂的“恢复窗口”。
 * 在这段时间内允许循迹控制更保守一些，避免刚回线时转向过猛。
 */
static uint8_t AppVehicle_IsLineRecovering(uint32_t now)
{
  return (uint8_t)((int32_t)(s_line_recover_until_tick - now) > 0);
}

/* 避障过程中临时占用 OLED 的状态行，提示当前避障阶段。 */
static void AppVehicle_ShowAvoidStatus(const uint8_t *text, uint8_t len)
{
  AppOled_ShowStatus(text, len);
  s_oled_showing_avoid = 1U;
}

/*
 * 定期通过串口上报码盘计数。
 * 主要用于调试电机、验证左右轮一致性，以及辅助闭环参数整定。
 */
static void AppVehicle_ReportEncoderCounts(uint32_t now)
{
#if ENABLE_ENCODER_UART_REPORT
  AppEncoder_Counts_t counts;
  char tx_buf[40];
  int len;

  if ((now - s_last_encoder_report_tick) < ENCODER_UART_REPORT_INTERVAL_MS)
  {
    return;
  }

  s_last_encoder_report_tick = now;
  counts = AppEncoder_GetCounts();
  len = snprintf(tx_buf,
                 sizeof(tx_buf),
                 "ENC L:%ld R:%ld\r\n",
                 (long)counts.left_count,
                 (long)counts.right_count);
  if (len > 0)
  {
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)tx_buf, (uint16_t)len, 20U);
  }
#else
  (void)now;
#endif
}

/* 避障文字显示结束后，把 OLED 恢复成目标站点信息。 */
static void AppVehicle_RestoreTargetStatus(void)
{
  if (s_oled_showing_avoid != 0U)
  {
    AppOled_ShowTarget(s_target_card);
    s_oled_showing_avoid = 0U;
  }
}

/*
 * 通过左右轮不同占空比实现转向。
 * 一侧快、一侧慢，车辆就会边前进边偏转。
 */
static void AppVehicle_SetAvoidTurn(uint8_t turn_left, uint16_t fast_pm, uint16_t slow_pm)
{
  if (turn_left != 0U)
  {
    AppMotor_SetTargetFromDuty(fast_pm, slow_pm);
  }
  else
  {
    AppMotor_SetTargetFromDuty(slow_pm, fast_pm);
  }
}

/*
 * 刷到有效 RFID 卡后启动任务。
 * A 卡对应第 1 条目标线，B 卡对应第 2 条目标线；
 * 到达目标线后停车取货，之后继续前往最终停靠线。
 */
static void AppVehicle_StartMission(uint8_t card_id)
{
  if ((card_id != CARD_ID_A) && (card_id != CARD_ID_B))
  {
    return;
  }

  s_target_card = card_id;
  s_target_stop_line = (uint8_t)((card_id == CARD_ID_A) ? 1U : 2U);
  s_final_stop_line = 3U;
  s_pickup_done = 0U;
  s_payload_raised = 0U;
  AppVehicle_ResetLineTracking();
  AppVehicle_ResetUltrasonicState();
  s_vehicle_state = VEHICLE_CARD_STANDBY;
  s_state_start_tick = HAL_GetTick();
  s_line_recover_until_tick = 0U;
  AppMotor_SetForwardDirection();
  AppVehicle_StopMotion();
  AppServo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
  AppOled_ShowTarget(card_id);
  AppOled_ClearAction();
  s_oled_showing_avoid = 0U;
}

/*
 * 任务结束或异常恢复后，回到待刷卡初始状态。
 * 这个过程不需要重启单片机，只重置任务上下文即可。
 */
static void AppVehicle_ResetMission(void)
{
  s_vehicle_state = VEHICLE_WAIT_CARD;
  s_target_card = CARD_ID_NONE;
  s_target_stop_line = 0U;
  s_final_stop_line = 0U;
  s_pickup_done = 0U;
  s_payload_raised = 0U;
  AppVehicle_ResetLineTracking();
  AppVehicle_ResetUltrasonicState();
  s_oled_showing_avoid = 0U;
  s_line_recover_until_tick = 0U;
#if ENABLE_MOTION_FEATURES
  AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
#endif
  AppOled_ClearAction();
}

/*
 * 仅在待机状态轮询 RFID。
 * 这样可以避免车辆运动过程中还频繁读卡，影响主流程时序，
 * 也能防止途中误刷卡打断当前任务。
 */
static void AppVehicle_ProcessRfid(uint32_t now)
{
#if ENABLE_NON_MOTION_FEATURES && ENABLE_RFID_FEATURES
  AppRfidEvent_t rfid_event;
  uint8_t allow_rfid_poll = (uint8_t)((AppUltrasonic_IsBusy() == 0U) &&
                                      (s_vehicle_state == VEHICLE_WAIT_CARD) &&
                                      (s_target_card == CARD_ID_NONE));

  if (AppRfid_Poll(now, allow_rfid_poll, &rfid_event) == 0U)
  {
    return;
  }

  if (rfid_event.is_new_uid == 0U)
  {
    return;
  }

  AppOled_ShowUid(rfid_event.uid, ENABLE_RFID_UART_REPORT);
  if ((s_vehicle_state == VEHICLE_WAIT_CARD) &&
      ((rfid_event.card_id == CARD_ID_A) || (rfid_event.card_id == CARD_ID_B)))
  {
    AppVehicle_StartMission(rfid_event.card_id);
  }
#else
  (void)now;
#endif
}

/*
 * 处理一次超声波结果，并做简单滤波。
 * 主要处理两类情况：
 * 1. 距离瞬间跳变过大，先尝试认为是异常值；
 * 2. 连续多次没有有效结果，则累计故障计数。
 */
static void AppVehicle_ProcessUltrasonicResult(void)
{
  uint16_t distance_cm;
  uint8_t has_distance = 0U;

  if (AppUltrasonic_FetchResult(&distance_cm, &has_distance) == 0U)
  {
    return;
  }

  if (has_distance != 0U)
  {
#if ENABLE_MOTION_FEATURES
    if ((s_has_last_distance != 0U) &&
        (AppVehicle_IsUltrasonicMotionState(s_vehicle_state) != 0U))
    {
      uint16_t diff_cm = (distance_cm > s_last_distance_cm) ?
        (uint16_t)(distance_cm - s_last_distance_cm) :
        (uint16_t)(s_last_distance_cm - distance_cm);

      if (diff_cm > HCSR04_MAX_JUMP_CM)
      {
        if ((distance_cm < s_last_distance_cm) && (distance_cm <= OBSTACLE_THRESHOLD_CM))
        {
          s_ultrasonic_jump_reject_count = 0U;
        }
        else if (s_ultrasonic_jump_reject_count < 2U)
        {
          s_ultrasonic_jump_reject_count++;
          distance_cm = s_last_distance_cm;
        }
        else
        {
          s_ultrasonic_jump_reject_count = 0U;
        }
      }
      else
      {
        s_ultrasonic_jump_reject_count = 0U;
      }
    }
    else
    {
      s_ultrasonic_jump_reject_count = 0U;
    }
#endif

    s_last_distance_cm = distance_cm;
    s_has_last_distance = 1U;
    s_ultrasonic_invalid_count = 0U;
  }
  else
  {
    s_has_last_distance = 0U;
#if ENABLE_MOTION_FEATURES
    if (AppVehicle_IsUltrasonicMotionState(s_vehicle_state) != 0U)
    {
      if (s_ultrasonic_invalid_count < 255U)
      {
        s_ultrasonic_invalid_count++;
      }
    }
    else
    {
      s_ultrasonic_invalid_count = 0U;
    }
#else
    s_ultrasonic_invalid_count = 0U;
#endif
  }
}

/*
 * 在主循环中轮询式驱动超声波模块。
 * 这里既负责推动一次次测量，也负责取回结果并更新整车状态。
 */
static void AppVehicle_PollUltrasonic(uint32_t now)
{
  uint32_t ultrasonic_interval_ms = HCSR04_MEASURE_INTERVAL_MS;
  uint8_t ultrasonic_enabled = 1U;

#if ENABLE_MOTION_FEATURES
  if (s_vehicle_state == VEHICLE_WAIT_CARD)
  {
    ultrasonic_enabled = 0U;
  }

  if (AppVehicle_IsUltrasonicMotionState(s_vehicle_state) != 0U)
  {
    ultrasonic_interval_ms = HCSR04_MOVE_INTERVAL_MS;
  }
#endif

  AppUltrasonic_Task(now);
  AppVehicle_ProcessUltrasonicResult();

  if ((ultrasonic_enabled != 0U) &&
      ((now - s_last_ultrasonic_poll_tick) >= ultrasonic_interval_ms))
  {
    s_last_ultrasonic_poll_tick = now;
    AppUltrasonic_StartMeasure();
  }
}

/*
 * 车辆主运行状态：循迹前进。
 * 这一状态下要同时做三件事：
 * 1. 根据红外循迹误差调整左右轮速度；
 * 2. 根据超声波距离判断是否进入避障；
 * 3. 统计整条黑线穿越次数，决定是否到站停车。
 */
static void AppVehicle_RunLineFollow(uint32_t now, uint8_t ir_mask)
{
  uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);
  uint8_t line_recovering = AppVehicle_IsLineRecovering(now);

  AppVehicle_RestoreTargetStatus();
  AppServo_SetAngle((uint16_t)((s_payload_raised != 0U) ? SERVO_STOP_END_ANGLE_DEG : SERVO_RUN_ANGLE_DEG));
  AppOled_ClearAction();

  if ((AppVehicle_IsUltrasonicFaulted() == 0U) &&
      (s_has_last_distance != 0U) &&
      (s_last_distance_cm <= OBSTACLE_THRESHOLD_CM))
  {
    s_vehicle_state = VEHICLE_AVOID_LEFT;
    s_avoid_line_seen_count = 0U;
    s_state_start_tick = now;
    return;
  }

  if ((now - s_last_line_pid_tick) < LINE_PID_INTERVAL_MS)
  {
    AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
    return;
  }

  {
    int16_t error_x10 = 0;
    int16_t correction_pm;
    int32_t duty_left_pm;
    int32_t duty_right_pm;

    s_last_line_pid_tick = now;

    if (AppLine_ComputeErrorX10(ir_mask, &error_x10) == 0U)
    {
      error_x10 = 0;
    }

    correction_pm = (int16_t)(error_x10 * LINE_KP_PM_PER_ERR10);
    if (line_recovering != 0U)
    {
      correction_pm = (int16_t)(correction_pm * LINE_RECOVER_KP_NUM);
    }
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

    AppMotor_SetEnable(1U);
    AppMotor_SetTargetFromDuty((uint16_t)duty_left_pm, (uint16_t)duty_right_pm);
  }

  if (line_recovering != 0U)
  {
    AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
    return;
  }

  if (s_line_cross_latched == 0U)
  {
    if (norm_mask != 0U)
    {
      if (s_line_black_frame_count < 255U)
      {
        s_line_black_frame_count++;
      }
      s_line_black_seen_mask |= norm_mask;

      if ((s_line_black_frame_count >= LINE_BLACK_CONFIRM_FRAMES) &&
          (s_line_black_seen_mask == 0x1FU))
      {
        s_crossed_line_count++;
        s_line_cross_latched = 1U;
        s_line_black_frame_count = 0U;
        s_line_black_seen_mask = 0U;

        if (s_pickup_done == 0U)
        {
          if ((s_target_stop_line != 0U) &&
              (s_crossed_line_count >= s_target_stop_line))
          {
            s_vehicle_state = VEHICLE_STOPPING;
            s_stop_start_tick = now;
            s_stop_is_final = 0U;
            AppVehicle_StopMotion();
          }
        }
        else if ((s_final_stop_line != 0U) && (s_crossed_line_count >= s_final_stop_line))
        {
          s_vehicle_state = VEHICLE_STOPPING;
          s_stop_start_tick = now;
          s_stop_is_final = 1U;
          AppVehicle_StopMotion();
        }
      }
    }
    else
    {
      s_line_black_frame_count = 0U;
      s_line_black_seen_mask = 0U;
    }
  }
  else if (norm_mask != 0x1FU)
  {
    s_line_cross_latched = 0U;
    s_line_black_frame_count = 0U;
    s_line_black_seen_mask = 0U;
  }

  AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
}

/*
 * 车辆停车状态。
 * 这里既承担“到目标点取货”的停车逻辑，
 * 也承担“到最终终点结束任务”的停车逻辑。
 */
static void AppVehicle_RunStopping(uint32_t now)
{
  uint32_t elapsed = now - s_stop_start_tick;

  AppVehicle_RestoreTargetStatus();
  AppVehicle_StopMotion();

  if (s_stop_is_final != 0U)
  {
    AppOled_ShowAction((const uint8_t *)"ARRIVE", 6U);
    if (elapsed < SERVO_HOLD_DURATION_MS)
    {
      AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
      return;
    }

    s_payload_raised = 0U;
    AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
    AppVehicle_ResetMission();
    return;
  }

  if (elapsed >= SERVO_HOLD_DURATION_MS)
  {
    s_pickup_done = 1U;
    s_payload_raised = 1U;
    s_line_black_frame_count = 0U;
    s_line_black_seen_mask = 0U;
    s_line_cross_latched = 0U;
    s_last_line_pid_tick = now;
    s_vehicle_state = VEHICLE_LINE_FOLLOW;
    return;
  }

  AppOled_ShowAction((const uint8_t *)"GET", 3U);
  AppServo_SetAngle(SERVO_STOP_END_ANGLE_DEG);
}

/*
 * 避障流程的最后阶段：向右搜索主线并重新并入循迹路径。
 * 如果探头重新看到边缘线，说明差不多已经回到轨道附近。
 */
static void AppVehicle_RunAvoidRight(uint32_t now, uint8_t ir_mask)
{
  uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);
  uint32_t elapsed = now - s_state_start_tick;

  AppMotor_SetEnable(1U);
  AppVehicle_SetAvoidTurn(0U, AVOID_RIGHT_DUTY_FAST_PM, AVOID_RIGHT_DUTY_SLOW_PM);
  AppVehicle_ShowAvoidStatus((const uint8_t *)"A:RLINE", 7U);
  AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);

  if ((norm_mask & AVOID_LINE_EDGE_MASK) != 0U)
  {
    if (s_avoid_line_seen_count < 255U)
    {
      s_avoid_line_seen_count++;
    }
  }
  else
  {
    s_avoid_line_seen_count = 0U;
  }

  if ((s_avoid_line_seen_count >= AVOID_LINE_CONFIRM_FRAMES) ||
      (elapsed >= AVOID_RIGHT_SEARCH_MAX_MS))
  {
    s_avoid_line_seen_count = 0U;
    s_line_black_frame_count = 0U;
    s_line_black_seen_mask = 0U;
    s_line_cross_latched = 0U;
    s_line_recover_until_tick = now + LINE_RECOVER_BOOST_MS;
    s_last_line_pid_tick = now;
    s_vehicle_state = VEHICLE_LINE_FOLLOW;
  }
}

/*
 * 整车状态机总入口。
 * 所有高层行为都从这里切换，包括：
 * - 待刷卡
 * - 刷卡后等待出发
 * - 正常循迹
 * - 到站停车
 * - 超声波避障的多个子阶段
 */
static void AppVehicle_RunStateMachine(uint32_t now, uint8_t ir_mask)
{
  switch (s_vehicle_state)
  {
    case VEHICLE_WAIT_CARD:
      AppVehicle_RestoreTargetStatus();
      AppVehicle_StopMotion();
      AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
      AppOled_ShowSwipePrompt();
      break;

    case VEHICLE_CARD_STANDBY:
      AppVehicle_RestoreTargetStatus();
      AppVehicle_StopMotion();
      AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
      if ((now - s_state_start_tick) >= CARD_STANDBY_DELAY_MS)
      {
        s_vehicle_state = VEHICLE_LINE_FOLLOW;
      }
      break;

    case VEHICLE_LINE_FOLLOW:
      AppVehicle_RunLineFollow(now, ir_mask);
      break;

    case VEHICLE_STOPPING:
      AppVehicle_RunStopping(now);
      break;

    case VEHICLE_AVOID_RIGHT:
      AppVehicle_RunAvoidRight(now, ir_mask);
      break;

    case VEHICLE_AVOID_LEFT:
      AppMotor_SetEnable(1U);
      AppVehicle_SetAvoidTurn(1U, AVOID_LEFT_DUTY_FAST_PM, AVOID_LEFT_DUTY_SLOW_PM);
      AppVehicle_ShowAvoidStatus((const uint8_t *)"A:LEFT", 6U);
      AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
      if ((now - s_state_start_tick) >= AVOID_LEFT_MS)
      {
        s_vehicle_state = VEHICLE_AVOID_FORWARD;
        s_state_start_tick = now;
      }
      break;

    case VEHICLE_AVOID_FORWARD:
      AppMotor_SetEnable(1U);
      AppMotor_SetTargetFromDuty(AVOID_FORWARD_DUTY_PM, AVOID_FORWARD_DUTY_PM);
      AppVehicle_ShowAvoidStatus((const uint8_t *)"A:FWD", 5U);
      AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
      if ((now - s_state_start_tick) >= AVOID_FORWARD_MS)
      {
        s_vehicle_state = VEHICLE_AVOID_RIGHT_ALIGN;
        s_state_start_tick = now;
      }
      break;

    case VEHICLE_AVOID_RIGHT_ALIGN:
      AppMotor_SetEnable(1U);
      AppVehicle_SetAvoidTurn(0U, AVOID_RIGHT_ALIGN_DUTY_FAST_PM, AVOID_RIGHT_ALIGN_DUTY_SLOW_PM);
      AppVehicle_ShowAvoidStatus((const uint8_t *)"A:RANG", 6U);
      AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
      if ((now - s_state_start_tick) >= AVOID_RIGHT_ALIGN_MS)
      {
        s_vehicle_state = VEHICLE_AVOID_RETURN_FORWARD;
        s_state_start_tick = now;
      }
      break;

    case VEHICLE_AVOID_RETURN_FORWARD:
      AppMotor_SetEnable(1U);
      AppMotor_SetTargetFromDuty(AVOID_RETURN_FORWARD_DUTY_PM, AVOID_RETURN_FORWARD_DUTY_PM);
      AppVehicle_ShowAvoidStatus((const uint8_t *)"A:RET", 5U);
      AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
      if ((now - s_state_start_tick) >= AVOID_RETURN_FORWARD_MS)
      {
        s_vehicle_state = VEHICLE_AVOID_RIGHT;
        s_avoid_line_seen_count = 0U;
        s_state_start_tick = now;
      }
      break;

    default:
      AppVehicle_ResetMission();
      break;
  }
}

/*
 * 初始化整车所依赖的所有模块。
 * 顺序上先初始化底层传感/执行模块，再初始化显示与任务上下文，
 * 最后把整车状态重置到待刷卡状态。
 */
void AppVehicle_Init(void)
{
  AppEncoder_Init();

#if ENABLE_NON_MOTION_FEATURES
  OLED_Init();
  AppOled_ShowSwipePrompt();
#if ENABLE_RFID_FEATURES
  AppRfid_Init();
#endif
#endif

#if ENABLE_MOTION_FEATURES
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  AppMotor_SetClosedLoop(ENABLE_MOTOR_CLOSED_LOOP);
  AppMotor_SetForwardDirection();
  AppMotor_SetEnable(0U);
#endif

  AppUltrasonic_Init(ENABLE_ULTRASONIC_UART_REPORT);
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0U);

#if ENABLE_MOTION_FEATURES
  AppServo_Init(SERVO_DEFAULT_ANGLE_DEG);
#endif

  s_last_ultrasonic_poll_tick = 0U;
  AppVehicle_ResetMission();
}

/*
 * 整车任务入口。
 * 每次被主循环调用时，都会读取当前时间和传感器状态，
 * 然后依次推进串口调试、RFID、超声波、状态机和电机控制。
 */
void AppVehicle_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t ir_mask = AppLine_ReadMask();

  AppVehicle_ReportEncoderCounts(now);

#if ENABLE_MOTION_FEATURES
  AppServo_Task();
#endif

  AppVehicle_ProcessRfid(now);
  AppVehicle_PollUltrasonic(now);

#if !ENABLE_MOTION_FEATURES
#if ENABLE_NON_MOTION_FEATURES
  AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
#endif
  return;
#endif

  AppVehicle_RunStateMachine(now, ir_mask);
  AppMotor_Task(now);
}
