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
#define LINE_BASE_DUTY_PM 160U
#define LINE_MIN_DUTY_PM 50U
#define LINE_MAX_DUTY_PM 650U
#define LINE_PID_INTERVAL_MS 10U
#define LINE_KP_PM_PER_ERR10 8
#define SERVO_RUN_ANGLE_DEG 55U
#define SERVO_DEFAULT_ANGLE_DEG SERVO_RUN_ANGLE_DEG
#define SERVO_ROTATE_DURATION_MS 2000U
#define SERVO_HOLD_DURATION_MS 2000U
#define SERVO_STOP_START_ANGLE_DEG SERVO_RUN_ANGLE_DEG
#define SERVO_STOP_END_ANGLE_DEG 90U
#define LINE_BLACK_CONFIRM_FRAMES 3U
#define OBSTACLE_THRESHOLD_CM 20U
#define AVOID_LEFT_MS 850U
#define AVOID_FORWARD_MS 180U
#define AVOID_RIGHT_ALIGN_MS 520U
#define AVOID_RETURN_FORWARD_MS 1400U
#define AVOID_RIGHT_SEARCH_MAX_MS 1600U
#define AVOID_LINE_CONFIRM_FRAMES 2U
#define AVOID_RIGHT_DUTY_FAST_PM 260U
#define AVOID_RIGHT_DUTY_SLOW_PM 50U
#define AVOID_LEFT_DUTY_FAST_PM 260U
#define AVOID_LEFT_DUTY_SLOW_PM 50U
#define AVOID_FORWARD_DUTY_PM 230U
#define AVOID_RETURN_FORWARD_DUTY_PM 230U
#define AVOID_RIGHT_ALIGN_DUTY_FAST_PM 260U
#define AVOID_RIGHT_ALIGN_DUTY_SLOW_PM 80U

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
static uint8_t s_crossed_line_count = 0U;
static uint8_t s_line_black_frame_count = 0U;
static uint8_t s_line_black_seen_mask = 0U;
static uint8_t s_line_cross_latched = 0U;
static uint8_t s_avoid_line_seen_count = 0U;
static uint8_t s_oled_showing_avoid = 0U;
static uint32_t s_state_start_tick = 0U;
static uint32_t s_stop_start_tick = 0U;
static uint8_t s_stop_at_next_line = 0U;

static uint8_t AppVehicle_IsUltrasonicMotionState(VehicleState_t state)
{
  return (uint8_t)((state == VEHICLE_LINE_FOLLOW) ||
                   (state == VEHICLE_AVOID_RIGHT) ||
                   (state == VEHICLE_AVOID_LEFT) ||
                   (state == VEHICLE_AVOID_FORWARD) ||
                   (state == VEHICLE_AVOID_RIGHT_ALIGN) ||
                   (state == VEHICLE_AVOID_RETURN_FORWARD));
}

static void AppVehicle_ResetUltrasonicState(void)
{
  s_last_distance_cm = 0U;
  s_has_last_distance = 0U;
  s_ultrasonic_jump_reject_count = 0U;
  s_ultrasonic_invalid_count = 0U;
}

static void AppVehicle_ResetLineTracking(void)
{
  s_crossed_line_count = 0U;
  s_line_black_frame_count = 0U;
  s_line_black_seen_mask = 0U;
  s_line_cross_latched = 0U;
  s_avoid_line_seen_count = 0U;
  s_last_line_pid_tick = 0U;
  s_stop_at_next_line = 0U;
}

static void AppVehicle_StopMotion(void)
{
  AppMotor_SetEnable(0U);
  AppMotor_SetDuty(0U, 0U);
}

static void AppVehicle_ShowAvoidStatus(const uint8_t *text, uint8_t len)
{
  AppOled_ShowStatus(text, len);
  s_oled_showing_avoid = 1U;
}

static void AppVehicle_RestoreTargetStatus(void)
{
  if (s_oled_showing_avoid != 0U)
  {
    AppOled_ShowTarget(s_target_card);
    s_oled_showing_avoid = 0U;
  }
}

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

static void AppVehicle_StartMission(uint8_t card_id)
{
  if ((card_id != CARD_ID_A) && (card_id != CARD_ID_B))
  {
    return;
  }

  s_target_card = card_id;
  s_target_stop_line = (card_id == CARD_ID_A) ? 1U : 2U;
  AppVehicle_ResetLineTracking();
  AppVehicle_ResetUltrasonicState();
  s_vehicle_state = VEHICLE_CARD_STANDBY;
  s_state_start_tick = HAL_GetTick();
  AppMotor_SetForwardDirection();
  AppVehicle_StopMotion();
  AppServo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
  AppOled_ShowTarget(card_id);
  s_oled_showing_avoid = 0U;
}

static void AppVehicle_ResetMission(void)
{
  s_vehicle_state = VEHICLE_WAIT_CARD;
  s_target_card = CARD_ID_NONE;
  s_target_stop_line = 0U;
  AppVehicle_ResetLineTracking();
  AppVehicle_ResetUltrasonicState();
  s_oled_showing_avoid = 0U;
#if ENABLE_MOTION_FEATURES
  AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
#endif
}

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

static void AppVehicle_RunLineFollow(uint32_t now, uint8_t ir_mask)
{
  uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);

  AppVehicle_RestoreTargetStatus();
  AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);

  if ((s_has_last_distance != 0U) && (s_last_distance_cm <= OBSTACLE_THRESHOLD_CM))
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

        if ((s_target_stop_line != 0U) && (s_crossed_line_count >= s_target_stop_line))
        {
          s_vehicle_state = VEHICLE_STOPPING;
          s_stop_start_tick = now;
          AppVehicle_StopMotion();

          if (s_stop_at_next_line != 0U)
          {
            s_target_stop_line = 0U;
          }
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

static void AppVehicle_RunStopping(uint32_t now)
{
  uint32_t elapsed = now - s_stop_start_tick;

  AppVehicle_RestoreTargetStatus();
  AppVehicle_StopMotion();

  if (s_stop_at_next_line != 0U)
  {
    AppVehicle_ResetMission();
    return;
  }

  if (elapsed >= (SERVO_ROTATE_DURATION_MS + SERVO_HOLD_DURATION_MS))
  {
    AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
    s_stop_at_next_line = 1U;
    s_target_stop_line = (uint8_t)(s_crossed_line_count + 1U);
    s_line_black_frame_count = 0U;
    s_line_black_seen_mask = 0U;
    s_line_cross_latched = 0U;
    s_last_line_pid_tick = now;
    s_vehicle_state = VEHICLE_LINE_FOLLOW;
    return;
  }

  if (elapsed >= SERVO_ROTATE_DURATION_MS)
  {
    AppServo_SetAngle(SERVO_STOP_END_ANGLE_DEG);
    return;
  }

  {
    int32_t angle_delta = (int32_t)SERVO_STOP_END_ANGLE_DEG - (int32_t)SERVO_STOP_START_ANGLE_DEG;
    uint16_t angle = (uint16_t)((int32_t)SERVO_STOP_START_ANGLE_DEG +
      ((angle_delta * (int32_t)elapsed) / (int32_t)SERVO_ROTATE_DURATION_MS));
    AppServo_SetAngle(angle);
  }
}

static void AppVehicle_RunAvoidRight(uint32_t now, uint8_t ir_mask)
{
  uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);
  uint32_t elapsed = now - s_state_start_tick;

  AppMotor_SetEnable(1U);
  AppVehicle_SetAvoidTurn(0U, AVOID_RIGHT_DUTY_FAST_PM, AVOID_RIGHT_DUTY_SLOW_PM);
  AppVehicle_ShowAvoidStatus((const uint8_t *)"AVOID:R-LINE", 12U);
  AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);

  if (norm_mask != 0U)
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
    s_last_line_pid_tick = now;
    s_vehicle_state = VEHICLE_LINE_FOLLOW;
  }
}

static void AppVehicle_RunStateMachine(uint32_t now, uint8_t ir_mask)
{
  if ((AppVehicle_IsUltrasonicMotionState(s_vehicle_state) != 0U) &&
      (s_ultrasonic_invalid_count >= HCSR04_INVALID_STOP_COUNT))
  {
    AppVehicle_StopMotion();
    AppVehicle_ShowAvoidStatus((const uint8_t *)"US:FAIL STOP", 12U);
    if (s_vehicle_state == VEHICLE_LINE_FOLLOW)
    {
      s_last_line_pid_tick = now;
    }
    else
    {
      s_state_start_tick = now;
    }
    AppOled_ShowDistance(s_has_last_distance, s_last_distance_cm);
    return;
  }

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
      AppVehicle_ShowAvoidStatus((const uint8_t *)"AVOID:LEFT", 10U);
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
      AppVehicle_ShowAvoidStatus((const uint8_t *)"AVOID:FWD", 9U);
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
      AppVehicle_ShowAvoidStatus((const uint8_t *)"AVOID:R-ALIGN", 13U);
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
      AppVehicle_ShowAvoidStatus((const uint8_t *)"AVOID:RETURN", 12U);
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

void AppVehicle_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t ir_mask = AppLine_ReadMask();

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
