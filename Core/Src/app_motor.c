#include "app_motor.h"

#include "app_encoder.h"
#include "main.h"
#include "tim.h"

#define MOTOR_CONTROL_INTERVAL_MS 20U /* 闭环控制周期（ms） */
#define MOTOR_MAX_TARGET_PPS 900       /* 目标速度上限（pulse per second） */
#define MOTOR_MIN_ACTIVE_DUTY_PM 40    /* 闭环运行时最小有效占空比（千分比） */
#define MOTOR_MAX_DUTY_PM 500          /* 闭环运行时最大占空比（千分比） */
#define MOTOR_KP_NUM 3                 /* PI 中比例项分子 */
#define MOTOR_KI_NUM 1                 /* PI 中积分项分子 */
#define MOTOR_GAIN_DEN 8               /* PI 总增益分母，用于整体缩放 */
#define MOTOR_INTEGRAL_LIMIT 3000      /* 积分项限幅，防止积分饱和 */
#define MOTOR_SWAP_PWM_OUTPUTS 1U      /* 是否交换左右PWM输出通道（用于线序适配） */
#define MOTOR_NO_FEEDBACK_LIMIT 10U    /* 连续无编码器反馈阈值，达到后停机保护 */

static uint8_t s_closed_loop_enable = 0U;      /* 闭环使能标志 */
static uint32_t s_last_control_tick = 0U;      /* 上一次闭环计算时间戳 */
static int32_t s_last_left_count = 0;          /* 上一次左编码器计数 */
static int32_t s_last_right_count = 0;         /* 上一次右编码器计数 */
static int16_t s_left_target_pps = 0;          /* 左轮目标速度 */
static int16_t s_right_target_pps = 0;         /* 右轮目标速度 */
static int32_t s_left_integral = 0;            /* 左轮积分项累计 */
static int32_t s_right_integral = 0;           /* 右轮积分项累计 */
static int32_t s_left_duty_pm = 0;             /* 左轮当前输出占空比命令 */
static int32_t s_right_duty_pm = 0;            /* 右轮当前输出占空比命令 */
static uint8_t s_left_no_feedback_count = 0U;  /* 左轮连续无反馈计数 */
static uint8_t s_right_no_feedback_count = 0U; /* 右轮连续无反馈计数 */

static int32_t AppMotor_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value; /* 小于下限则钳位到下限 */
  }
  if (value > max_value)
  {
    return max_value; /* 大于上限则钳位到上限 */
  }
  return value;       /* 在范围内原样返回 */
}

static void AppMotor_ResetLoopState(void)
{
  AppEncoder_Counts_t counts = AppEncoder_GetCounts();

  s_last_control_tick = HAL_GetTick();    /* 重置控制节拍起点 */
  s_last_left_count = counts.left_count;  /* 以当前计数作为新基准 */
  s_last_right_count = counts.right_count;
  s_left_integral = 0;                    /* 清空积分状态 */
  s_right_integral = 0;
  s_left_duty_pm = 0;                     /* 清空输出状态 */
  s_right_duty_pm = 0;
  s_left_no_feedback_count = 0U;          /* 清空保护计数器 */
  s_right_no_feedback_count = 0U;
}

void AppMotor_SetEnable(uint8_t enable)
{
  if (enable != 0U)
  {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET); /* 拉高 STBY 使能电机驱动芯片 */
    return;
  }

  s_left_target_pps = 0;                                  /* 停机时清空速度目标 */
  s_right_target_pps = 0;
  AppMotor_SetDuty(0U, 0U);                              /* 先把 PWM 输出置零 */
  AppMotor_ResetLoopState();                             /* 再清闭环内部状态 */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET); /* 最后拉低 STBY 关断驱动 */
}

void AppMotor_SetForwardDirection(void)
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);    /* A桥方向：前进 */
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);    /* B桥方向：前进 */
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

void AppMotor_SetDuty(uint16_t duty_left_pm, uint16_t duty_right_pm)
{
  uint32_t arr = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  uint16_t ccr1;
  uint16_t ccr2;

  if (duty_left_pm > 1000U)
  {
    duty_left_pm = 1000U; /* 限幅到 100% */
  }
  if (duty_right_pm > 1000U)
  {
    duty_right_pm = 1000U; /* 限幅到 100% */
  }

  ccr1 = (uint16_t)(((arr + 1U) * duty_left_pm) / 1000U);  /* 千分比转定时器比较值 */
  ccr2 = (uint16_t)(((arr + 1U) * duty_right_pm) / 1000U); /* 千分比转定时器比较值 */

#if MOTOR_SWAP_PWM_OUTPUTS
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr2); /* 交换通道输出，适配电机接线 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr1);
#else
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1); /* 正常映射：CH1=左，CH2=右 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
#endif
}

void AppMotor_SetClosedLoop(uint8_t enable)
{
  s_closed_loop_enable = (enable != 0U) ? 1U : 0U; /* 统一成 0/1 */
  AppMotor_ResetLoopState();                       /* 切模式后重置状态，避免突变 */
}

void AppMotor_SetTargetSpeed(int16_t left_pulse_per_sec, int16_t right_pulse_per_sec)
{
  s_left_target_pps = left_pulse_per_sec;   /* 设置左轮目标速度 */
  s_right_target_pps = right_pulse_per_sec; /* 设置右轮目标速度 */
}

void AppMotor_SetTargetFromDuty(uint16_t duty_left_pm, uint16_t duty_right_pm)
{
  if (duty_left_pm > 1000U)
  {
    duty_left_pm = 1000U; /* 输入限幅 */
  }
  if (duty_right_pm > 1000U)
  {
    duty_right_pm = 1000U; /* 输入限幅 */
  }

  s_left_target_pps = (int16_t)(((int32_t)duty_left_pm * MOTOR_MAX_TARGET_PPS) / 1000);   /* 占空比映射为目标速度 */
  s_right_target_pps = (int16_t)(((int32_t)duty_right_pm * MOTOR_MAX_TARGET_PPS) / 1000); /* 占空比映射为目标速度 */

  if (s_closed_loop_enable == 0U)
  {
    AppMotor_SetDuty(duty_left_pm, duty_right_pm); /* 开环模式下直接输出占空比 */
  }
}

void AppMotor_Task(uint32_t now_ms)
{
  AppEncoder_Counts_t counts;
  uint32_t elapsed_ms;
  int32_t left_delta;
  int32_t right_delta;
  int32_t left_speed_pps;
  int32_t right_speed_pps;
  int32_t left_error;
  int32_t right_error;

  if (s_closed_loop_enable == 0U)
  {
    return; /* 开环模式不执行闭环计算 */
  }

  elapsed_ms = now_ms - s_last_control_tick;
  if (elapsed_ms < MOTOR_CONTROL_INTERVAL_MS)
  {
    return; /* 未到控制周期则跳过 */
  }

  counts = AppEncoder_GetCounts();
  left_delta = counts.left_count - s_last_left_count;
  right_delta = counts.right_count - s_last_right_count;
  s_last_left_count = counts.left_count;
  s_last_right_count = counts.right_count;
  s_last_control_tick = now_ms;

  left_speed_pps = (left_delta * 1000) / (int32_t)elapsed_ms;   /* 编码器增量换算速度 */
  right_speed_pps = (right_delta * 1000) / (int32_t)elapsed_ms; /* 编码器增量换算速度 */

  if ((s_left_target_pps > 0) && (left_delta == 0) && (s_left_duty_pm > 200))
  {
    if (s_left_no_feedback_count < 255U)
    {
      s_left_no_feedback_count++; /* 有目标且有较大驱动但无编码器变化，记一次异常 */
    }
  }
  else
  {
    s_left_no_feedback_count = 0U; /* 条件不满足则清零 */
  }

  if ((s_right_target_pps > 0) && (right_delta == 0) && (s_right_duty_pm > 200))
  {
    if (s_right_no_feedback_count < 255U)
    {
      s_right_no_feedback_count++; /* 右轮同样的失反馈检测 */
    }
  }
  else
  {
    s_right_no_feedback_count = 0U; /* 条件不满足则清零 */
  }

  if ((s_left_no_feedback_count >= MOTOR_NO_FEEDBACK_LIMIT) ||
      (s_right_no_feedback_count >= MOTOR_NO_FEEDBACK_LIMIT))
  {
    AppMotor_SetDuty(0U, 0U);   /* 保护触发：立即停机 */
    AppMotor_ResetLoopState();  /* 清状态，等待上层重新下发命令 */
    return;
  }

  left_error = (int32_t)s_left_target_pps - left_speed_pps;   /* 左轮速度误差 */
  right_error = (int32_t)s_right_target_pps - right_speed_pps; /* 右轮速度误差 */

  s_left_integral = AppMotor_ClampI32(s_left_integral + left_error,
                                      -MOTOR_INTEGRAL_LIMIT,
                                      MOTOR_INTEGRAL_LIMIT); /* 积分累加并限幅 */
  s_right_integral = AppMotor_ClampI32(s_right_integral + right_error,
                                       -MOTOR_INTEGRAL_LIMIT,
                                       MOTOR_INTEGRAL_LIMIT); /* 积分累加并限幅 */

  s_left_duty_pm += ((left_error * MOTOR_KP_NUM) + (s_left_integral * MOTOR_KI_NUM)) / MOTOR_GAIN_DEN;   /* PI 更新左轮输出 */
  s_right_duty_pm += ((right_error * MOTOR_KP_NUM) + (s_right_integral * MOTOR_KI_NUM)) / MOTOR_GAIN_DEN; /* PI 更新右轮输出 */

  if (s_left_target_pps <= 0)
  {
    s_left_duty_pm = 0; /* 无目标速度则直接输出0 */
    s_left_integral = 0; /* 同时清积分避免残留 */
  }
  else
  {
    s_left_duty_pm = AppMotor_ClampI32(s_left_duty_pm, MOTOR_MIN_ACTIVE_DUTY_PM, MOTOR_MAX_DUTY_PM); /* 运行时限幅 */
  }

  if (s_right_target_pps <= 0)
  {
    s_right_duty_pm = 0; /* 无目标速度则直接输出0 */
    s_right_integral = 0; /* 同时清积分避免残留 */
  }
  else
  {
    s_right_duty_pm = AppMotor_ClampI32(s_right_duty_pm, MOTOR_MIN_ACTIVE_DUTY_PM, MOTOR_MAX_DUTY_PM); /* 运行时限幅 */
  }

  AppMotor_SetDuty((uint16_t)s_left_duty_pm, (uint16_t)s_right_duty_pm); /* 写入最终 PWM 输出 */
}
