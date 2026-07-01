/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mini forno v2: DHT11, MPU6050, OLED, UART ESP32, proporcional e PWM
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "ssd1306_safe.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim3;

#ifdef HAL_IWDG_MODULE_ENABLED
IWDG_HandleTypeDef hiwdg;
#endif

/* USER CODE BEGIN PD */
#define MPU6050_ADDR                (0x68U << 1)
#define MPU6050_ACCEL_XOUT_H        0x3BU
#define MPU6050_PWR_MGMT_1          0x6BU
#define MPU6050_GYRO_CONFIG         0x1BU
#define MPU6050_WHO_AM_I            0x75U

#define HEATER_PWM_FREQ_HZ          1000U
#define PWM_MAX_PERCENT             100U
#define SOFT_STEP_MS                100U
#define SOFT_STEP_DUTY              20U
#define CONTROL_PERIOD_MS           100U
#define UART_PERIOD_MS              100U
#define LOG_PERIOD_MS               30000U
#define UART_RX_BUFFER_SIZE         128U
#define BUTTON_DEBOUNCE_MS          45U
#define BUTTON_LONG_MS              1500U
#define MENU_SHOW_MS                3500U
#define CONFIG_AUTOSAVE_MS          5000U

#define DEFAULT_SETPOINT_X10        PROFILE_TIN_TARGET_X10
#define DEFAULT_HEATING_SECONDS     10UL
#define DEFAULT_ANGLE_LIMIT_X10     150L
#define DEFAULT_KP_X100             300L
#define DEFAULT_SAFETY_RETURN       1UL
#define DEFAULT_SOAK_SECONDS        10UL
#define PROFILE_TIN_TARGET_X10      300L
#define PROFILE_BRASS_TARGET_X10    400L
#define PROFILE_ALUMINUM_TARGET_X10 350L
#define PROFILE_MANUAL_TARGET_X10   350L
#define PREHEATING_TEMP_X10         280L
#define HEATING_STEP_SECONDS        10L
#define SOAK_STEP_SECONDS           10L
#define TARGET_STEP_X10             10L

#define TEMP_SAFE_X10               300L
#define TEMP_CUTOFF_X10             600L
#define FLASH_CFG_ADDR              0x08060000UL
#define FLASH_CFG_MAGIC             0x4D463256UL
#define FLASH_CFG_VERSION           3UL
/* USER CODE END PD */

/* USER CODE BEGIN PTD */
typedef enum
{
  OVEN_START = 0,
  OVEN_CONFIGURATION,
  OVEN_START_CONFIRMATION,
  OVEN_PREHEATING,
  OVEN_HEATING,
  OVEN_SOAKING,
  OVEN_COOLING,
  OVEN_FINISHED,
  OVEN_MANUAL_MODE,
  OVEN_ERROR
} OvenState_t;

typedef enum
{
  PROFILE_TIN = 0,
  PROFILE_BRASS,
  PROFILE_ALUMINUM,
  PROFILE_MANUAL
} MaterialProfile_t;

typedef enum
{
  CONFIG_STEP_PROFILE = 0,
  CONFIG_STEP_TARGET,
  CONFIG_STEP_HEATING_TIME,
  CONFIG_STEP_SOAKING_TIME,
  CONFIG_STEP_TOTAL
} ConfigStep_t;

typedef enum
{
  MENU_PROFILE = 0,
  MENU_TARGET,
  MENU_HEATING_TIME,
  MENU_SOAKING_TIME,
  MENU_DOOR_SAFETY,
  MENU_ANGLE,
  MENU_GAIN,
  MENU_TOTAL
} MenuItem_t;

typedef enum
{
  SRC_BOOT = 0,
  SRC_STM32,
  SRC_WEB,
  SRC_SYSTEM,
  SRC_SAFETY
} SyncSource_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t profile;
  int32_t setpoint_x10;
  uint32_t heating_seconds;
  int32_t angle_limit_x10;
  int32_t kp_x100;
  int32_t hysteresis_x10;
  uint32_t safety_return_to_start;
  uint32_t soak_seconds;
  uint32_t crc;
} OvenConfig_t;

typedef struct
{
  int32_t temperature_x10;
  int32_t setpoint_x10;
  int32_t door_angle_x10;
  int32_t door_angle_y10;
  int32_t door_limit_x10;
  uint8_t heater_enabled;
  uint8_t heater_output_percent;
  uint8_t heater_relay_on;
  uint8_t door_is_open;
  uint8_t safety_return_to_start;
  uint8_t emergency_active;
  uint8_t emergency_auto;
  uint8_t manual_mode;
  uint8_t progress_percent;
  uint8_t mpu_ok;
  uint8_t mpu_calibrated;
  uint8_t led_ready;
  uint8_t led_heating;
  uint8_t led_holding;
  uint8_t led_done;
  uint8_t led_emergency;
  uint8_t led_pwm_mirror;
  uint8_t led_pwm_level;
  uint8_t led_heartbeat;
  uint32_t seconds;
  OvenState_t current_state;
} OvenData_t;
/* USER CODE END PTD */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
#ifdef HAL_IWDG_MODULE_ENABLED
static void MX_IWDG_Init(void);
#endif

/* USER CODE BEGIN PFP */
static void DWT_DelayInit(void);
static void DelayUs(uint32_t us);
static void DHT11_SetOutput(void);
static void DHT11_SetInput(void);
static uint8_t DHT11_WaitForState(GPIO_PinState state, uint32_t timeout_us);
static int32_t DHT11_ReadTempX10(void);
static uint8_t MPU6050_Init(void);
static uint8_t MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
static uint8_t MPU6050_CalibrateZero(void);
static int32_t MPU6050_ApproxAngleDegX10(int16_t accel_value, int16_t zero_value);
static void Config_Defaults(void);
static uint32_t Config_Crc(const OvenConfig_t *cfg);
static void Config_Load(void);
static uint8_t Config_Save(void);
static void Config_MarkDirty(void);
static void Config_MarkDirtyFrom(SyncSource_t source);
static void Sync_MarkConfigFrom(SyncSource_t source);
static void Sync_MarkStateFrom(SyncSource_t source);
static const char *Sync_SourceName(SyncSource_t source);
static void Telemetry_ForceUpdate(void);
static void Config_AutoSave(uint32_t now);
static void Config_NextMenu(void);
static void Config_Adjust(int8_t direction);
static const char *Config_MenuName(MenuItem_t item);
static const char *Config_StepName(ConfigStep_t step);
static const char *Profile_Name(MaterialProfile_t profile);
static void Profile_Apply(MaterialProfile_t profile);
static void Profile_ApplyFrom(MaterialProfile_t profile, SyncSource_t source);
static void Config_Back(void);
static void Config_Enter(void);
static void Oven_SetState(OvenState_t new_state);
static void Oven_SetStateFrom(OvenState_t new_state, SyncSource_t source);
static const char *Oven_StateName(OvenState_t state);
static const char *Oven_ModeName(OvenState_t state);
static void Oven_Start(void);
static void Oven_Stop(void);
static void Oven_ClearEmergency(int32_t temp_x10);
static void Oven_UpdateState(int32_t temp_x10, int32_t ang_x10, int32_t ang_y10, uint8_t mpu_read_ok);
static void state_inicio(void);
static void state_configuracao(void);
static void state_confirmar(void);
static void state_pre_aquecimento(int32_t temp_x10);
static void state_aquecendo(int32_t temp_x10);
static void state_patamar(int32_t temp_x10);
static void state_resfriando(int32_t temp_x10);
static void state_finalizado(int32_t temp_x10);
static void state_manual(void);
static void state_emergencia(void);
static uint8_t Door_IsOpen(int32_t ang_x10, int32_t ang_y10, uint8_t mpu_read_ok);
static uint8_t Door_SafetyCanTrigger(OvenState_t state);
static void OvenData_Update(int32_t temp_x10, int32_t ang_x10, int32_t ang_y10, uint8_t mpu_read_ok, uint32_t now);
static uint8_t Proportional_ComputeDuty(int32_t temp_x10);
static void Heater_UpdateDuty(int32_t temp_x10, uint32_t now);
static void Heater_SetPower(uint8_t percent);
static void MirrorLed_SetPower(uint8_t percent);
static void Outputs_SetPower(uint8_t percent);
static void Outputs_ForceOff(void);
static void LEDs_Update(uint32_t now);
static uint8_t Oven_Progress(uint32_t now, int32_t temp_x10);
static void OLED_ShowScreen(const OvenData_t *data);
static void Buttons_Poll(uint32_t now);
static void Button_Check(GPIO_TypeDef *port, uint16_t pin, uint8_t *stable, uint8_t *last_raw, uint32_t *last_change, const char *press_cmd, uint32_t now);
static uint8_t Command_Process(const char *cmd);
static uint8_t Command_ProcessFrom(const char *cmd, SyncSource_t source);
static uint8_t Command_SetStateByName(const char *name);
static void UART_PollCommands(void);
static void UART_StartReceive(void);
static void UART_SendStatus(void);
static void UART_SendAck(const char *cmd);
static void UART_SendError(const char *reason);
static void UART_SendLog(int32_t temp_x10);
static int32_t ClampLong(int32_t value, int32_t min_value, int32_t max_value);
static int32_t AbsLong(int32_t value);
/* USER CODE END PFP */

/* USER CODE BEGIN PV */
static uint8_t oled_ok = 0U;
static uint8_t mpu_ok = 0U;
static uint8_t mpu_calibrated = 0U;
static uint8_t emergency_latched = 0U;
static uint8_t emergency_auto = 0U;
static uint8_t heater_duty = 0U;
static uint8_t heater_target_duty = 0U;
static uint8_t heater_output = 0U;
static uint8_t led_pwm_level = 0U;
static uint8_t telemetry_force_update = 1U;
static uint8_t manual_mode = 0U;
static uint8_t manual_duty = 0U;
static uint8_t state_entry = 1U;
static uint8_t soak_timer_running = 0U;
static uint8_t cfg_dirty = 0U;
static uint8_t btn_plus_stable = 0U;
static uint8_t btn_minus_stable = 0U;
static uint8_t btn_enter_stable = 0U;
static uint8_t btn_plus_last_raw = 0U;
static uint8_t btn_minus_last_raw = 0U;
static uint8_t btn_enter_last_raw = 0U;
static uint8_t enter_long_sent = 0U;
static const char *command_error_reason = "UNKNOWN_COMMAND";
static SyncSource_t active_command_source = SRC_SYSTEM;
static uint32_t cfg_rev = 0U;
static uint32_t state_rev = 0U;
static uint32_t sync_rev = 0U;
static SyncSource_t cfg_source = SRC_BOOT;
static SyncSource_t state_source = SRC_BOOT;
static SyncSource_t sync_source = SRC_BOOT;
static int16_t accel_x_zero = 0;
static int16_t accel_y_zero = 0;
static OvenData_t oven_data;
static OvenConfig_t cfg;
static OvenState_t oven_state = OVEN_START;
static MenuItem_t menu_item = MENU_PROFILE;
static ConfigStep_t config_step = CONFIG_STEP_PROFILE;
static uint32_t seconds_on = 0U;
static uint32_t state_enter_ms = 0U;
static uint32_t process_start_ms = 0U;
static uint32_t soak_start_ms = 0U;
static uint32_t last_led_ms = 0U;
static uint32_t last_control_ms = 0U;
static uint32_t last_uart_ms = 0U;
static uint32_t last_log_ms = 0U;
static uint32_t last_soft_ms = 0U;
static uint32_t cfg_last_change_ms = 0U;
static uint32_t menu_show_until_ms = 0U;
static uint32_t btn_plus_last_change = 0U;
static uint32_t btn_minus_last_change = 0U;
static uint32_t btn_enter_last_change = 0U;
static uint32_t enter_press_ms = 0U;
static uint8_t uart_rx_line[80];
static uint8_t uart_rx_index = 0U;
static uint8_t uart_rx_byte = 0U;
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_head = 0U;
static volatile uint16_t uart_rx_tail = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static int32_t ClampLong(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static int32_t AbsLong(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static void DWT_DelayInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DelayUs(uint32_t us)
{
  uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * cycles_per_us;

  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

static void DHT11_SetOutput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

static void DHT11_SetInput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t DHT11_WaitForState(GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = timeout_us * (HAL_RCC_GetHCLKFreq() / 1000000U);

  while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) != state)
  {
    if ((DWT->CYCCNT - start) > cycles)
    {
      return 0U;
    }
  }
  return 1U;
}

static int32_t DHT11_ReadTempX10(void)
{
  static int32_t cached_temp_x10 = -1L;
  static uint32_t last_read_ms = 0U;
  uint8_t data[5] = {0};
  uint32_t now = HAL_GetTick();

  if ((cached_temp_x10 >= 0L) && ((now - last_read_ms) < 2000U))
  {
    return cached_temp_x10;
  }

  DHT11_SetOutput();
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
  DelayUs(30U);
  DHT11_SetInput();

  if (DHT11_WaitForState(GPIO_PIN_RESET, 100U) == 0U)
  {
    return cached_temp_x10;
  }
  if (DHT11_WaitForState(GPIO_PIN_SET, 100U) == 0U)
  {
    return cached_temp_x10;
  }
  if (DHT11_WaitForState(GPIO_PIN_RESET, 100U) == 0U)
  {
    return cached_temp_x10;
  }

  for (uint8_t bit = 0U; bit < 40U; bit++)
  {
    uint32_t high_start;
    uint32_t high_us;

    if (DHT11_WaitForState(GPIO_PIN_SET, 80U) == 0U)
    {
      return cached_temp_x10;
    }

    high_start = DWT->CYCCNT;

    if (DHT11_WaitForState(GPIO_PIN_RESET, 120U) == 0U)
    {
      return cached_temp_x10;
    }

    high_us = (DWT->CYCCNT - high_start) / (HAL_RCC_GetHCLKFreq() / 1000000U);
    data[bit / 8U] <<= 1U;
    if (high_us > 45U)
    {
      data[bit / 8U] |= 1U;
    }
  }

  if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
  {
    return cached_temp_x10;
  }

  cached_temp_x10 = ((int32_t)data[2] * 10L) + (int32_t)data[3];
  last_read_ms = now;
  return cached_temp_x10;
}

static uint8_t MPU6050_Init(void)
{
  uint8_t value;
  uint8_t who = 0U;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who, 1U, 100U) != HAL_OK)
  {
    return 0U;
  }

  value = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return 0U;
  }

  HAL_Delay(50U);

  value = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t data[6];

  if ((ax == NULL) || (ay == NULL) || (az == NULL))
  {
    return 0U;
  }

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 100U) != HAL_OK)
  {
    return 0U;
  }

  *ax = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  *ay = (int16_t)((uint16_t)data[2] << 8 | data[3]);
  *az = (int16_t)((uint16_t)data[4] << 8 | data[5]);
  return 1U;
}

static uint8_t MPU6050_CalibrateZero(void)
{
  int32_t sx = 0L;
  int32_t sy = 0L;
  int16_t ax;
  int16_t ay;
  int16_t az;

  for (uint8_t i = 0U; i < 16U; i++)
  {
    if (MPU6050_ReadAccel(&ax, &ay, &az) == 0U)
    {
      return 0U;
    }
    sx += ax;
    sy += ay;
    HAL_Delay(5U);
  }

  accel_x_zero = (int16_t)(sx / 16L);
  accel_y_zero = (int16_t)(sy / 16L);
  mpu_calibrated = 1U;
  return 1U;
}

static int32_t MPU6050_ApproxAngleDegX10(int16_t accel_value, int16_t zero_value)
{
  int32_t delta = (int32_t)accel_value - (int32_t)zero_value;
  int32_t angle_x10 = (delta * 900L) / 16384L;

  return ClampLong(angle_x10, -900L, 900L);
}

static void Config_Defaults(void)
{
  cfg.magic = FLASH_CFG_MAGIC;
  cfg.version = FLASH_CFG_VERSION;
  cfg.profile = PROFILE_TIN;
  cfg.setpoint_x10 = DEFAULT_SETPOINT_X10;
  cfg.heating_seconds = DEFAULT_HEATING_SECONDS;
  cfg.angle_limit_x10 = DEFAULT_ANGLE_LIMIT_X10;
  cfg.kp_x100 = DEFAULT_KP_X100;
  cfg.hysteresis_x10 = 0L;
  cfg.safety_return_to_start = DEFAULT_SAFETY_RETURN;
  cfg.soak_seconds = DEFAULT_SOAK_SECONDS;
  cfg.crc = Config_Crc(&cfg);
}

static uint32_t Config_Crc(const OvenConfig_t *item)
{
  const uint32_t *words = (const uint32_t *)item;
  uint32_t crc = 0xA5A55A5AU;

  for (uint32_t i = 0U; i < ((sizeof(OvenConfig_t) / 4U) - 1U); i++)
  {
    crc ^= words[i] + (0x9E3779B9UL + (crc << 6) + (crc >> 2));
  }

  return crc;
}

static void Config_Load(void)
{
  const OvenConfig_t *saved = (const OvenConfig_t *)FLASH_CFG_ADDR;

  Config_Defaults();

  if ((saved->magic == FLASH_CFG_MAGIC) &&
      (saved->version == FLASH_CFG_VERSION) &&
      (saved->crc == Config_Crc(saved)))
  {
    cfg = *saved;
  }

  if (cfg.profile > PROFILE_MANUAL)
  {
    cfg.profile = PROFILE_TIN;
  }
  cfg.setpoint_x10 = ClampLong(cfg.setpoint_x10, 0L, 1200L);
  if ((cfg.heating_seconds < 5UL) || (cfg.heating_seconds > 3600UL))
  {
    cfg.heating_seconds = DEFAULT_HEATING_SECONDS;
  }
  cfg.angle_limit_x10 = ClampLong(cfg.angle_limit_x10, 20L, 900L);
  cfg.kp_x100 = ClampLong(cfg.kp_x100, 0L, 1000L);
  cfg.hysteresis_x10 = ClampLong(cfg.hysteresis_x10, 0L, 100L);
  cfg.safety_return_to_start = (cfg.safety_return_to_start != 0UL) ? 1UL : 0UL;
  if ((cfg.soak_seconds < 5UL) || (cfg.soak_seconds > 3600UL))
  {
    cfg.soak_seconds = DEFAULT_SOAK_SECONDS;
  }
}

static uint8_t Config_Save(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;
  const uint32_t *words;
  uint32_t address = FLASH_CFG_ADDR;
  OvenConfig_t save = cfg;

  save.magic = FLASH_CFG_MAGIC;
  save.version = FLASH_CFG_VERSION;
  save.crc = Config_Crc(&save);

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_7;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  words = (const uint32_t *)&save;
  for (uint32_t i = 0U; i < (sizeof(OvenConfig_t) / 4U); i++)
  {
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, address, words[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return 0U;
    }
    address += 4U;
  }

  HAL_FLASH_Lock();
  cfg = save;
  return 1U;
}

static void Config_MarkDirty(void)
{
  Config_MarkDirtyFrom(SRC_STM32);
}

static void Config_MarkDirtyFrom(SyncSource_t source)
{
  Sync_MarkConfigFrom(source);
}

static void Sync_MarkConfigFrom(SyncSource_t source)
{
  cfg_dirty = 1U;
  cfg_rev++;
  sync_rev++;
  cfg_source = source;
  sync_source = source;
  cfg_last_change_ms = HAL_GetTick();
  menu_show_until_ms = cfg_last_change_ms + MENU_SHOW_MS;
  Telemetry_ForceUpdate();
}

static void Sync_MarkStateFrom(SyncSource_t source)
{
  state_rev++;
  sync_rev++;
  state_source = source;
  sync_source = source;
  Telemetry_ForceUpdate();
}

static const char *Sync_SourceName(SyncSource_t source)
{
  switch (source)
  {
    case SRC_STM32:  return "STM32";
    case SRC_WEB:    return "WEB";
    case SRC_SYSTEM: return "SYSTEM";
    case SRC_SAFETY: return "SAFETY";
    case SRC_BOOT:
    default:         return "BOOT";
  }
}

static void Telemetry_ForceUpdate(void)
{
  telemetry_force_update = 1U;
}

static void Config_AutoSave(uint32_t now)
{
  if ((cfg_dirty != 0U) && ((now - cfg_last_change_ms) >= CONFIG_AUTOSAVE_MS))
  {
    if (Config_Save() != 0U)
    {
      cfg_dirty = 0U;
    }
  }
}

static const char *Config_MenuName(MenuItem_t item)
{
  switch (item)
  {
    case MENU_PROFILE:      return "MATERIAL";
    case MENU_TARGET:       return "TEMP";
    case MENU_HEATING_TIME: return "AQUEC";
    case MENU_SOAKING_TIME: return "PATAMAR";
    case MENU_DOOR_SAFETY:  return "SEG_PORTA";
    case MENU_ANGLE:        return "ANGULO";
    case MENU_GAIN:         return "GANHO_P";
    default:                return "MENU";
  }
}

static const char *Config_StepName(ConfigStep_t step)
{
  switch (step)
  {
    case CONFIG_STEP_PROFILE:      return "MATERIAL";
    case CONFIG_STEP_TARGET:       return "TEMPERATURA";
    case CONFIG_STEP_HEATING_TIME: return "TEMPO_AQUEC";
    case CONFIG_STEP_SOAKING_TIME: return "PATAMAR";
    default:                       return "CONFIG";
  }
}

static const char *Profile_Name(MaterialProfile_t profile)
{
  switch (profile)
  {
    case PROFILE_TIN:      return "ESTANHO";
    case PROFILE_BRASS:    return "LATAO";
    case PROFILE_ALUMINUM: return "ALUMINIO";
    case PROFILE_MANUAL:   return "MANUAL";
    default:               return "ESTANHO";
  }
}

static void Profile_Apply(MaterialProfile_t profile)
{
  Profile_ApplyFrom(profile, SRC_STM32);
}

static void Profile_ApplyFrom(MaterialProfile_t profile, SyncSource_t source)
{
  cfg.profile = (uint32_t)profile;
  cfg.heating_seconds = DEFAULT_HEATING_SECONDS;
  cfg.soak_seconds = DEFAULT_SOAK_SECONDS;

  switch (profile)
  {
    case PROFILE_TIN:
      cfg.setpoint_x10 = PROFILE_TIN_TARGET_X10;
      break;

    case PROFILE_BRASS:
      cfg.setpoint_x10 = PROFILE_BRASS_TARGET_X10;
      break;

    case PROFILE_ALUMINUM:
      cfg.setpoint_x10 = PROFILE_ALUMINUM_TARGET_X10;
      break;

    case PROFILE_MANUAL:
    default:
      cfg.setpoint_x10 = PROFILE_MANUAL_TARGET_X10;
      break;
  }

  Config_MarkDirtyFrom(source);
}

static void Config_NextMenu(void)
{
  if (oven_state == OVEN_CONFIGURATION)
  {
    Config_Enter();
    return;
  }

  menu_item = (MenuItem_t)((uint8_t)menu_item + 1U);
  if (menu_item >= MENU_TOTAL)
  {
    menu_item = MENU_PROFILE;
  }
  menu_show_until_ms = HAL_GetTick() + MENU_SHOW_MS;
  Telemetry_ForceUpdate();
}

static void Config_Adjust(int8_t direction)
{
  int32_t step;
  uint8_t changed_by_profile = 0U;

  if (direction == 0)
  {
    return;
  }

  if (oven_state == OVEN_CONFIGURATION)
  {
    switch (config_step)
    {
      case CONFIG_STEP_PROFILE:      menu_item = MENU_PROFILE; break;
      case CONFIG_STEP_TARGET:       menu_item = MENU_TARGET; break;
      case CONFIG_STEP_HEATING_TIME: menu_item = MENU_HEATING_TIME; break;
      case CONFIG_STEP_SOAKING_TIME: menu_item = MENU_SOAKING_TIME; break;
      default:                       menu_item = MENU_PROFILE; break;
    }
  }

  switch (menu_item)
  {
    case MENU_PROFILE:
      if (direction > 0)
      {
        Profile_ApplyFrom((MaterialProfile_t)((cfg.profile + 1UL) % 4UL), active_command_source);
      }
      else
      {
        cfg.profile = (cfg.profile == 0UL) ? PROFILE_MANUAL : (cfg.profile - 1UL);
        Profile_ApplyFrom((MaterialProfile_t)cfg.profile, active_command_source);
      }
      changed_by_profile = 1U;
      break;

    case MENU_TARGET:
      step = TARGET_STEP_X10 * (int32_t)direction;
      cfg.setpoint_x10 = ClampLong(cfg.setpoint_x10 + step, 0L, 1200L);
      break;

    case MENU_ANGLE:
      step = 10L * (int32_t)direction;
      cfg.angle_limit_x10 = ClampLong(cfg.angle_limit_x10 + step, 20L, 900L);
      break;

    case MENU_HEATING_TIME:
      step = HEATING_STEP_SECONDS * (int32_t)direction;
      cfg.heating_seconds = (uint32_t)ClampLong((int32_t)cfg.heating_seconds + step, 5L, 3600L);
      break;

    case MENU_SOAKING_TIME:
      step = SOAK_STEP_SECONDS * (int32_t)direction;
      cfg.soak_seconds = (uint32_t)ClampLong((int32_t)cfg.soak_seconds + step, 5L, 3600L);
      break;

    case MENU_DOOR_SAFETY:
      cfg.safety_return_to_start = (cfg.safety_return_to_start == 0UL) ? 1UL : 0UL;
      break;

    case MENU_GAIN:
      step = 10L * (int32_t)direction;
      cfg.kp_x100 = ClampLong(cfg.kp_x100 + step, 0L, 1000L);
      break;

    default:
      break;
  }

  if (changed_by_profile == 0U)
  {
    Config_MarkDirtyFrom(active_command_source);
  }
}

static void Config_Back(void)
{
  if (oven_state == OVEN_CONFIGURATION)
  {
    if (config_step == CONFIG_STEP_PROFILE)
    {
      Oven_SetState(OVEN_START);
    }
    else
    {
      config_step = (ConfigStep_t)((uint8_t)config_step - 1U);
      Sync_MarkStateFrom(active_command_source);
    }
  }
  else if (oven_state == OVEN_START_CONFIRMATION)
  {
    Oven_SetState(OVEN_CONFIGURATION);
    config_step = CONFIG_STEP_SOAKING_TIME;
    Sync_MarkStateFrom(active_command_source);
  }

  menu_show_until_ms = HAL_GetTick() + MENU_SHOW_MS;
  Telemetry_ForceUpdate();
}

static void Config_Enter(void)
{
  if (oven_state == OVEN_START)
  {
    config_step = CONFIG_STEP_PROFILE;
    Oven_SetState(OVEN_CONFIGURATION);
    return;
  }

  if (oven_state == OVEN_CONFIGURATION)
  {
    if (config_step < (CONFIG_STEP_TOTAL - 1U))
    {
      config_step = (ConfigStep_t)((uint8_t)config_step + 1U);
      Sync_MarkStateFrom(active_command_source);
    }
    else
    {
      Oven_SetState(OVEN_START_CONFIRMATION);
    }
    menu_show_until_ms = HAL_GetTick() + MENU_SHOW_MS;
    return;
  }

  if (oven_state == OVEN_START_CONFIRMATION)
  {
    Oven_Start();
  }
}

static void Oven_SetStateFrom(OvenState_t new_state, SyncSource_t source)
{
  SyncSource_t previous_source = active_command_source;

  active_command_source = source;
  Oven_SetState(new_state);
  active_command_source = previous_source;
}

static void Oven_SetState(OvenState_t new_state)
{
  if (oven_state == new_state)
  {
    return;
  }

  oven_state = new_state;
  state_enter_ms = HAL_GetTick();
  state_entry = 1U;
  Sync_MarkStateFrom(active_command_source);

  if (new_state != OVEN_MANUAL_MODE)
  {
    manual_mode = 0U;
    manual_duty = 0U;
  }

  if ((new_state == OVEN_PREHEATING) || (new_state == OVEN_HEATING))
  {
    process_start_ms = state_enter_ms;
    soak_start_ms = 0U;
    soak_timer_running = 0U;
    heater_duty = 0U;
    heater_target_duty = 0U;
    last_soft_ms = state_enter_ms;
  }
  else if (new_state == OVEN_SOAKING)
  {
    soak_start_ms = 0U;
    soak_timer_running = 0U;
  }
  else if ((new_state == OVEN_START) || (new_state == OVEN_CONFIGURATION) ||
           (new_state == OVEN_START_CONFIRMATION) || (new_state == OVEN_FINISHED) ||
           (new_state == OVEN_COOLING) || (new_state == OVEN_ERROR))
  {
    soak_start_ms = 0U;
    soak_timer_running = 0U;
    heater_target_duty = 0U;
    Outputs_ForceOff();
  }

  if (new_state == OVEN_CONFIGURATION)
  {
    menu_item = MENU_PROFILE;
  }
  else if (new_state == OVEN_MANUAL_MODE)
  {
    manual_mode = 1U;
    manual_duty = 0U;
  }
}

static const char *Oven_StateName(OvenState_t state)
{
  switch (state)
  {
    case OVEN_START:              return "INICIO";
    case OVEN_CONFIGURATION:      return "CONFIG";
    case OVEN_START_CONFIRMATION: return "CONFIRMAR";
    case OVEN_PREHEATING:         return "PRE_AQUEC";
    case OVEN_HEATING:            return "AQUECENDO";
    case OVEN_SOAKING:            return "PATAMAR";
    case OVEN_COOLING:            return "RESFRIANDO";
    case OVEN_FINISHED:           return "FINALIZADO";
    case OVEN_MANUAL_MODE:        return "MANUAL";
    case OVEN_ERROR:              return "EMERGENCIA";
    default:                      return "ERRO";
  }
}

static const char *Oven_ModeName(OvenState_t state)
{
  switch (state)
  {
    case OVEN_START:              return "STOP";
    case OVEN_CONFIGURATION:      return "CONFIG";
    case OVEN_START_CONFIRMATION: return "CONFIG";
    case OVEN_PREHEATING:         return "RUN";
    case OVEN_HEATING:            return "RUN";
    case OVEN_SOAKING:            return "RUN";
    case OVEN_COOLING:            return "COOLING";
    case OVEN_FINISHED:           return "DONE";
    case OVEN_MANUAL_MODE:        return "MANUAL";
    case OVEN_ERROR:              return "EMERGENCY";
    default:                      return "UNKNOWN";
  }
}

static void Oven_Start(void)
{
  manual_mode = 0U;
  manual_duty = 0U;
  emergency_auto = 0U;
  if (emergency_latched == 0U)
  {
    heater_target_duty = 0U;
    Outputs_ForceOff();
    Oven_SetState(OVEN_PREHEATING);
  }
  else
  {
    Outputs_ForceOff();
  }
}

static void Oven_Stop(void)
{
  manual_mode = 0U;
  manual_duty = 0U;
  heater_target_duty = 0U;
  Outputs_ForceOff();
  Oven_SetState(OVEN_COOLING);
}

static void Oven_ClearEmergency(int32_t temp_x10)
{
  emergency_latched = 0U;
  emergency_auto = 0U;
  manual_mode = 0U;
  manual_duty = 0U;
  heater_target_duty = 0U;
  Outputs_ForceOff();

  if (temp_x10 > TEMP_SAFE_X10)
  {
    Oven_SetState(OVEN_COOLING);
  }
  else
  {
    Oven_SetState(OVEN_START);
  }
}

static void Oven_UpdateState(int32_t temp_x10, int32_t ang_x10, int32_t ang_y10, uint8_t mpu_read_ok)
{
  oven_data.door_is_open = Door_IsOpen(ang_x10, ang_y10, mpu_read_ok);

  if ((oven_data.door_is_open != 0U) &&
      (cfg.safety_return_to_start != 0UL) &&
      (Door_SafetyCanTrigger(oven_state) != 0U))
  {
    if (emergency_latched == 0U)
    {
      Sync_MarkStateFrom(SRC_SAFETY);
    }
    emergency_latched = 1U;
    emergency_auto = 1U;
  }

  if ((temp_x10 >= TEMP_CUTOFF_X10) && (temp_x10 >= 0L))
  {
    if (emergency_latched == 0U)
    {
      Sync_MarkStateFrom(SRC_SAFETY);
    }
    emergency_latched = 1U;
    emergency_auto = 1U;
  }

  if (emergency_latched != 0U)
  {
    manual_mode = 0U;
    manual_duty = 0U;
    Outputs_ForceOff();
    Oven_SetStateFrom(OVEN_ERROR, SRC_SAFETY);
  }

  if (manual_mode != 0U)
  {
    return;
  }

  switch (oven_state)
  {
    case OVEN_START:
      state_inicio();
      break;

    case OVEN_CONFIGURATION:
      state_configuracao();
      break;

    case OVEN_START_CONFIRMATION:
      state_confirmar();
      break;

    case OVEN_PREHEATING:
      state_pre_aquecimento(temp_x10);
      break;

    case OVEN_HEATING:
      state_aquecendo(temp_x10);
      break;

    case OVEN_SOAKING:
      state_patamar(temp_x10);
      break;

    case OVEN_FINISHED:
      state_finalizado(temp_x10);
      break;

    case OVEN_COOLING:
      state_resfriando(temp_x10);
      break;

    case OVEN_MANUAL_MODE:
      state_manual();
      break;

    case OVEN_ERROR:
      state_emergencia();
      break;

    default:
      break;
  }
}

static void state_inicio(void)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    heater_target_duty = 0U;
    heater_duty = 0U;
    manual_mode = 0U;
    manual_duty = 0U;
    state_entry = 0U;
    flag = 1U;
  }

  if ((seconds_on - timePast) >= 2UL)
  {
    config_step = CONFIG_STEP_PROFILE;
    Oven_SetState(OVEN_CONFIGURATION);
    flag = 0U;
  }
}

static void state_configuracao(void)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    heater_target_duty = 0U;
    heater_duty = 0U;
    manual_mode = 0U;
    manual_duty = 0U;
    state_entry = 0U;
    flag = 1U;
  }

  switch (config_step)
  {
    case CONFIG_STEP_PROFILE:      menu_item = MENU_PROFILE; break;
    case CONFIG_STEP_TARGET:       menu_item = MENU_TARGET; break;
    case CONFIG_STEP_HEATING_TIME: menu_item = MENU_HEATING_TIME; break;
    case CONFIG_STEP_SOAKING_TIME: menu_item = MENU_SOAKING_TIME; break;
    default:                       menu_item = MENU_PROFILE; break;
  }

  (void)timePast;
}

static void state_confirmar(void)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    heater_target_duty = 0U;
    heater_duty = 0U;
    manual_mode = 0U;
    manual_duty = 0U;
    state_entry = 0U;
    flag = 1U;
  }

  (void)timePast;
}

static void state_pre_aquecimento(int32_t temp_x10)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    process_start_ms = HAL_GetTick();
    state_entry = 0U;
    flag = 1U;
  }

  if ((temp_x10 >= PREHEATING_TEMP_X10) || ((seconds_on - timePast) >= 30UL))
  {
    Oven_SetState(OVEN_HEATING);
    flag = 0U;
  }
}

static void state_aquecendo(int32_t temp_x10)
{
  static uint8_t flag = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    process_start_ms = HAL_GetTick();
    soak_start_ms = 0U;
    soak_timer_running = 0U;
    state_entry = 0U;
    flag = 1U;
  }

  if (manual_mode != 0U)
  {
    return;
  }

  if (temp_x10 < 0L)
  {
    heater_target_duty = 0U;
    return;
  }

  if (temp_x10 >= cfg.setpoint_x10)
  {
    Oven_SetState(OVEN_SOAKING);
    flag = 0U;
  }
}

static void state_patamar(int32_t temp_x10)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed_seconds = 0U;

  if (state_entry != 0U)
  {
    soak_start_ms = 0U;
    soak_timer_running = 0U;
    state_entry = 0U;
  }

  if (manual_mode != 0U)
  {
    return;
  }

  /* The soak timer only accumulates while the oven is at or above the setpoint. */
  if ((temp_x10 < 0L) || (temp_x10 < cfg.setpoint_x10))
  {
    soak_start_ms = 0U;
    soak_timer_running = 0U;
    return;
  }

  if (soak_timer_running == 0U)
  {
    soak_start_ms = now;
    soak_timer_running = 1U;
  }

  elapsed_seconds = (now - soak_start_ms) / 1000UL;
  if (elapsed_seconds >= cfg.soak_seconds)
  {
    soak_start_ms = 0U;
    soak_timer_running = 0U;

    if (temp_x10 > TEMP_SAFE_X10)
    {
      Oven_SetState(OVEN_COOLING);
    }
    else
    {
      Oven_SetState(OVEN_FINISHED);
    }
  }
}

static void state_resfriando(int32_t temp_x10)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    heater_target_duty = 0U;
    heater_duty = 0U;
    manual_mode = 0U;
    manual_duty = 0U;
    state_entry = 0U;
    flag = 1U;
  }

  if ((temp_x10 >= 0L) && (temp_x10 <= TEMP_SAFE_X10))
  {
    Oven_SetState(OVEN_FINISHED);
    flag = 0U;
  }

  (void)timePast;
}

static void state_finalizado(int32_t temp_x10)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    heater_target_duty = 0U;
    heater_duty = 0U;
    manual_mode = 0U;
    manual_duty = 0U;
    state_entry = 0U;
    flag = 1U;
  }

  if (temp_x10 > TEMP_SAFE_X10)
  {
    Oven_SetState(OVEN_COOLING);
    flag = 0U;
  }

  (void)timePast;
}

static void state_manual(void)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    manual_mode = 1U;
    heater_target_duty = manual_duty;
    Outputs_SetPower(manual_duty);
    state_entry = 0U;
    flag = 1U;
  }

  (void)timePast;
}

static void state_emergencia(void)
{
  static uint8_t flag = 0U;
  static uint32_t timePast = 0U;

  if ((!flag) || (state_entry != 0U))
  {
    timePast = seconds_on;
    heater_target_duty = 0U;
    heater_duty = 0U;
    manual_mode = 0U;
    manual_duty = 0U;
    Outputs_ForceOff();
    state_entry = 0U;
    flag = 1U;
  }

  (void)timePast;
}

static uint8_t Door_IsOpen(int32_t ang_x10, int32_t ang_y10, uint8_t mpu_read_ok)
{
  if ((mpu_read_ok == 0U) || (mpu_calibrated == 0U))
  {
    return 0U;
  }

  return ((AbsLong(ang_x10) > cfg.angle_limit_x10) ||
          (AbsLong(ang_y10) > cfg.angle_limit_x10)) ? 1U : 0U;
}

static uint8_t Door_SafetyCanTrigger(OvenState_t state)
{
  return ((state == OVEN_PREHEATING) ||
          (state == OVEN_HEATING) ||
          (state == OVEN_SOAKING) ||
          (state == OVEN_MANUAL_MODE)) ? 1U : 0U;
}

static void OvenData_Update(int32_t temp_x10, int32_t ang_x10, int32_t ang_y10, uint8_t mpu_read_ok, uint32_t now)
{
  oven_data.temperature_x10 = temp_x10;
  oven_data.setpoint_x10 = cfg.setpoint_x10;
  oven_data.door_angle_x10 = ang_x10;
  oven_data.door_angle_y10 = ang_y10;
  oven_data.door_limit_x10 = cfg.angle_limit_x10;
  oven_data.door_is_open = Door_IsOpen(ang_x10, ang_y10, mpu_read_ok);
  oven_data.heater_enabled = (heater_duty > 0U) ? 1U : 0U;
  oven_data.heater_output_percent = heater_duty;
  oven_data.heater_relay_on = (heater_duty > 0U) ? 1U : 0U;
  oven_data.safety_return_to_start = (uint8_t)(cfg.safety_return_to_start != 0UL);
  oven_data.emergency_active = emergency_latched;
  oven_data.emergency_auto = emergency_auto;
  oven_data.manual_mode = manual_mode;
  oven_data.progress_percent = Oven_Progress(now, temp_x10);
  oven_data.mpu_ok = mpu_read_ok;
  oven_data.mpu_calibrated = mpu_calibrated;
  oven_data.led_ready = (HAL_GPIO_ReadPin(LED_READY_GPIO_Port, LED_READY_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  oven_data.led_heating = (HAL_GPIO_ReadPin(LED_HEATING_GPIO_Port, LED_HEATING_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  oven_data.led_holding = (HAL_GPIO_ReadPin(LED_HOLDING_GPIO_Port, LED_HOLDING_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  oven_data.led_done = (HAL_GPIO_ReadPin(LED_DONE_GPIO_Port, LED_DONE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  oven_data.led_emergency = (HAL_GPIO_ReadPin(LED_EMERGENCY_GPIO_Port, LED_EMERGENCY_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  oven_data.led_pwm_mirror = (led_pwm_level > 0U) ? 1U : 0U;
  oven_data.led_pwm_level = led_pwm_level;
  oven_data.led_heartbeat = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1U : 0U;
  oven_data.seconds = seconds_on;
  oven_data.current_state = oven_state;
}

static uint8_t Proportional_ComputeDuty(int32_t temp_x10)
{
  int32_t error;
  int32_t output;

  if (temp_x10 < 0L)
  {
    return 0U;
  }

  error = cfg.setpoint_x10 - temp_x10;
  if (error <= 0L)
  {
    return 0U;
  }

  output = (cfg.kp_x100 * error) / 1000L;

  output = ClampLong(output, 0L, 100L);
  return (uint8_t)output;
}

static void Heater_UpdateDuty(int32_t temp_x10, uint32_t now)
{
  uint8_t target = 0U;

  if ((emergency_latched != 0U) || (oven_state == OVEN_ERROR) ||
      ((oven_data.door_is_open != 0U) &&
       (cfg.safety_return_to_start != 0UL) &&
       (Door_SafetyCanTrigger(oven_state) != 0U)) ||
      ((temp_x10 >= TEMP_CUTOFF_X10) && (temp_x10 >= 0L)))
  {
    heater_target_duty = 0U;
    Outputs_ForceOff();
    return;
  }

  if (manual_mode != 0U)
  {
    heater_target_duty = manual_duty;
    Outputs_SetPower(manual_duty);
    return;
  }

  if ((oven_state == OVEN_PREHEATING) || (oven_state == OVEN_HEATING) || (oven_state == OVEN_SOAKING))
  {
    target = Proportional_ComputeDuty(temp_x10);
  }

  heater_target_duty = target;
  Outputs_SetPower(target);
  (void)now;
}

static void Heater_SetPower(uint8_t percent)
{
  uint8_t previous_duty = heater_duty;
  uint32_t arr;
  uint32_t compare;

  if (percent > PWM_MAX_PERCENT)
  {
    percent = PWM_MAX_PERCENT;
  }

  arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  compare = ((arr + 1U) * (uint32_t)percent) / PWM_MAX_PERCENT;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, compare);

  heater_duty = percent;
  heater_output = (percent > 0U) ? 1U : 0U;

  if (heater_duty != previous_duty)
  {
    Telemetry_ForceUpdate();
  }
}

static void MirrorLed_SetPower(uint8_t percent)
{
  uint8_t previous_level = led_pwm_level;
  uint32_t arr;
  uint32_t compare;

  if (percent > PWM_MAX_PERCENT)
  {
    percent = PWM_MAX_PERCENT;
  }

  arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  compare = ((arr + 1U) * (uint32_t)percent) / PWM_MAX_PERCENT;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);

  led_pwm_level = percent;

  if (led_pwm_level != previous_level)
  {
    Telemetry_ForceUpdate();
  }
}

static void Outputs_SetPower(uint8_t percent)
{
  if (emergency_latched != 0U)
  {
    percent = 0U;
  }

  Heater_SetPower(percent);
  MirrorLed_SetPower(percent);
}

static void Outputs_ForceOff(void)
{
  Heater_SetPower(0U);
  MirrorLed_SetPower(0U);
}

static void LEDs_Update(uint32_t now)
{
  uint8_t blink_fast = ((now / 100U) % 2U) != 0U;
  uint8_t blink_slow = ((now / 500U) % 2U) != 0U;

  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin,
                    ((oven_state == OVEN_START) || (oven_state == OVEN_CONFIGURATION) ||
                     (oven_state == OVEN_START_CONFIRMATION)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HEATING_GPIO_Port, LED_HEATING_Pin,
                    ((oven_state == OVEN_PREHEATING) || (oven_state == OVEN_HEATING)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HOLDING_GPIO_Port, LED_HOLDING_Pin,
                    ((oven_state == OVEN_SOAKING) || (oven_state == OVEN_MANUAL_MODE)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_DONE_GPIO_Port, LED_DONE_Pin,
                    ((oven_state == OVEN_FINISHED) || (oven_state == OVEN_COOLING)) && blink_slow ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_EMERGENCY_GPIO_Port, LED_EMERGENCY_Pin,
                    (oven_state == OVEN_ERROR) && blink_fast ? GPIO_PIN_SET : GPIO_PIN_RESET);

  last_led_ms = now;
}

static uint8_t Oven_Progress(uint32_t now, int32_t temp_x10)
{
  if (oven_state == OVEN_ERROR)
  {
    return (((now / 500U) % 2U) != 0U) ? 100U : 0U;
  }

  switch (oven_state)
  {
    case OVEN_START:
      return 0U;

    case OVEN_CONFIGURATION:
      return (uint8_t)(10U + ((uint8_t)config_step * 8U));

    case OVEN_START_CONFIRMATION:
      return 38U;

    case OVEN_PREHEATING:
      return 45U;

    case OVEN_HEATING:
      if (cfg.heating_seconds > 0UL)
      {
        uint32_t elapsed = (now - state_enter_ms) / 1000UL;
        uint32_t pct = 45UL + ((elapsed * 25UL) / cfg.heating_seconds);
        if (pct > 70UL)
        {
          pct = 70UL;
        }
        return (uint8_t)pct;
      }
      else if ((temp_x10 > 0L) && (cfg.setpoint_x10 > 0L))
      {
        int32_t pct = (temp_x10 * 70L) / cfg.setpoint_x10;
        return (uint8_t)ClampLong(pct, 10L, 70L);
      }
      return 55U;

    case OVEN_SOAKING:
      if ((cfg.soak_seconds > 0UL) && (soak_timer_running != 0U))
      {
        uint32_t elapsed = (now - soak_start_ms) / 1000UL;
        uint32_t pct = 70UL + ((elapsed * 25UL) / cfg.soak_seconds);
        if (pct > 95UL)
        {
          pct = 95UL;
        }
        return (uint8_t)pct;
      }
      return 70U;

    case OVEN_FINISHED:
      return 100U;

    case OVEN_COOLING:
      return 95U;

    case OVEN_MANUAL_MODE:
      return (uint8_t)manual_duty;

    default:
      return 0U;
  }
}

static void OLED_ShowScreen(const OvenData_t *data)
{
  char line[22];
  uint32_t now = HAL_GetTick();
  uint8_t fill;

  if (data == NULL)
  {
    return;
  }

  fill = (uint8_t)((data->progress_percent * 118U) / 100U);

  SSD1306_ClearBuffer();

  snprintf(line, sizeof(line), "SP:%ld.%ldC PWM:%u%%",
           data->setpoint_x10 / 10L, AbsLong(data->setpoint_x10 % 10L),
           (unsigned int)data->heater_output_percent);
  SSD1306_PrintNoUpdate(0U, 0U, line);

  if (data->temperature_x10 >= 0L)
  {
    snprintf(line, sizeof(line), "TEMP:%ld.%ldC",
             data->temperature_x10 / 10L,
             AbsLong(data->temperature_x10 % 10L));
  }
  else
  {
    snprintf(line, sizeof(line), "TEMP: ERRO");
  }
  SSD1306_PrintNoUpdate(1U, 0U, line);

  snprintf(line, sizeof(line), "PWR:%u%% HEAT:%s",
           (unsigned int)data->heater_output_percent,
           (data->heater_relay_on != 0U) ? "ON" : "OFF");
  SSD1306_PrintNoUpdate(2U, 0U, line);

  snprintf(line, sizeof(line), "STATE:%s", Oven_StateName(data->current_state));
  SSD1306_PrintNoUpdate(3U, 0U, line);

  if ((now < menu_show_until_ms) || (oven_state == OVEN_CONFIGURATION))
  {
    switch (menu_item)
    {
      case MENU_PROFILE:
        snprintf(line, sizeof(line), "%s", Profile_Name((MaterialProfile_t)cfg.profile));
        break;

      case MENU_TARGET:
        snprintf(line, sizeof(line), "%s %ld.%ldC", Config_MenuName(menu_item),
                 cfg.setpoint_x10 / 10L, cfg.setpoint_x10 % 10L);
        break;

      case MENU_ANGLE:
        snprintf(line, sizeof(line), "%s %ld.%ld", Config_MenuName(menu_item),
                 cfg.angle_limit_x10 / 10L, cfg.angle_limit_x10 % 10L);
        break;

      case MENU_HEATING_TIME:
        snprintf(line, sizeof(line), "%s %lus", Config_MenuName(menu_item),
                 cfg.heating_seconds);
        break;

      case MENU_SOAKING_TIME:
        snprintf(line, sizeof(line), "%s %lus", Config_MenuName(menu_item),
                 cfg.soak_seconds);
        break;

      case MENU_DOOR_SAFETY:
        snprintf(line, sizeof(line), "%s %s", Config_MenuName(menu_item),
                 (cfg.safety_return_to_start != 0UL) ? "ON" : "OFF");
        break;

      case MENU_GAIN:
        snprintf(line, sizeof(line), "%s %ld.%02ld", Config_MenuName(menu_item),
                 cfg.kp_x100 / 100L, cfg.kp_x100 % 100L);
        break;

      default:
        snprintf(line, sizeof(line), "MENU");
        break;
    }
    SSD1306_PrintNoUpdate(4U, 0U, line);
  }

  SSD1306_DrawRect(4U, 50U, 120U, 10U, 1U);
  if (fill > 0U)
  {
    SSD1306_FillRect(5U, 51U, fill, 8U, 1U);
  }

  (void)SSD1306_Update();
}

static void Button_Check(GPIO_TypeDef *port, uint16_t pin, uint8_t *stable, uint8_t *last_raw,
                         uint32_t *last_change, const char *press_cmd, uint32_t now)
{
  uint8_t raw_pressed = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if (raw_pressed != *last_raw)
  {
    *last_raw = raw_pressed;
    *last_change = now;
  }

  if (((now - *last_change) >= BUTTON_DEBOUNCE_MS) && (raw_pressed != *stable))
  {
    *stable = raw_pressed;

    if (*stable != 0U)
    {
      if (press_cmd != NULL)
      {
        (void)Command_ProcessFrom(press_cmd, SRC_STM32);
      }
    }
  }
}

static void Buttons_Poll(uint32_t now)
{
  Button_Check(BTN_PLUS_GPIO_Port, BTN_PLUS_Pin,
               &btn_plus_stable, &btn_plus_last_raw, &btn_plus_last_change,
               "PLUS", now);

  Button_Check(BTN_MINUS_GPIO_Port, BTN_MINUS_Pin,
               &btn_minus_stable, &btn_minus_last_raw, &btn_minus_last_change,
               "MINUS", now);

  Button_Check(BTN_ENTER_GPIO_Port, BTN_ENTER_Pin,
               &btn_enter_stable, &btn_enter_last_raw, &btn_enter_last_change,
               NULL, now);

  if ((btn_enter_stable != 0U) && (enter_press_ms == 0U))
  {
    enter_press_ms = now;
    enter_long_sent = 0U;
  }

  if ((btn_enter_stable != 0U) && (enter_long_sent == 0U) &&
      ((now - enter_press_ms) >= BUTTON_LONG_MS))
  {
    (void)Command_ProcessFrom("NEXTCFG", SRC_STM32);
    enter_long_sent = 1U;
  }

  if ((btn_enter_stable == 0U) && (enter_press_ms != 0U))
  {
    if (enter_long_sent == 0U)
    {
      (void)Command_ProcessFrom("ENTER", SRC_STM32);
    }
    enter_press_ms = 0U;
    enter_long_sent = 0U;
  }
}

static uint8_t Command_SetStateByName(const char *name)
{
  if ((emergency_latched != 0U) &&
      (strcmp(name, "EMERGENCIA") != 0) &&
      (strcmp(name, "EMERGENCY") != 0))
  {
    command_error_reason = "EMERGENCY_ACTIVE";
    return 0U;
  }

  if ((strcmp(name, "INICIO") == 0) || (strcmp(name, "START") == 0) ||
      (strcmp(name, "PRONTO") == 0) || (strcmp(name, "READY") == 0))
  {
    Oven_SetState(OVEN_START);
  }
  else if ((strcmp(name, "CONFIG") == 0) || (strcmp(name, "CONFIGURACAO") == 0))
  {
    Oven_SetState(OVEN_CONFIGURATION);
  }
  else if ((strcmp(name, "CONFIRMAR") == 0) || (strcmp(name, "CONFIRM") == 0))
  {
    Oven_SetState(OVEN_START_CONFIRMATION);
  }
  else if ((strcmp(name, "PRE_AQUEC") == 0) || (strcmp(name, "PREHEATING") == 0))
  {
    Oven_SetState(OVEN_PREHEATING);
  }
  else if ((strcmp(name, "AQUECENDO") == 0) || (strcmp(name, "HEATING") == 0))
  {
    Oven_SetState(OVEN_HEATING);
  }
  else if ((strcmp(name, "PATAMAR") == 0) || (strcmp(name, "MANTENDO") == 0) ||
           (strcmp(name, "SOAKING") == 0) || (strcmp(name, "HOLDING") == 0))
  {
    Oven_SetState(OVEN_SOAKING);
  }
  else if ((strcmp(name, "RESFRIANDO") == 0) || (strcmp(name, "COOLING") == 0))
  {
    Oven_SetState(OVEN_COOLING);
  }
  else if ((strcmp(name, "FINALIZADO") == 0) || (strcmp(name, "DONE") == 0))
  {
    Oven_SetState(OVEN_FINISHED);
  }
  else if ((strcmp(name, "MANUAL") == 0) || (strcmp(name, "MANUAL_MODE") == 0))
  {
    Oven_SetState(OVEN_MANUAL_MODE);
  }
  else if ((strcmp(name, "EMERGENCIA") == 0) || (strcmp(name, "EMERGENCY") == 0))
  {
    emergency_latched = 1U;
    emergency_auto = 0U;
    Outputs_ForceOff();
    Oven_SetStateFrom(OVEN_ERROR, SRC_SAFETY);
  }
  else
  {
    return 0U;
  }

  return 1U;
}

static uint8_t Command_ProcessFrom(const char *cmd, SyncSource_t source)
{
  uint8_t ok;
  SyncSource_t previous_source = active_command_source;

  active_command_source = source;
  ok = Command_Process(cmd);
  active_command_source = previous_source;

  return ok;
}

static uint8_t Command_Process(const char *cmd)
{
  long value;
  uint8_t valid = 1U;

  command_error_reason = "UNKNOWN_COMMAND";

  if (cmd == NULL)
  {
    return 0U;
  }

  if (strcmp(cmd, "GET") == 0)
  {
    /* Force telemetry only. */
  }
  else if (strcmp(cmd, "START") == 0)
  {
    if (emergency_latched != 0U)
    {
      command_error_reason = "EMERGENCY_ACTIVE";
      valid = 0U;
    }
    else
    {
      Oven_Start();
    }
  }
  else if (strcmp(cmd, "STOP") == 0)
  {
    Oven_Stop();
  }
  else if (strcmp(cmd, "RESET") == 0)
  {
    emergency_latched = 0U;
    emergency_auto = 0U;
    manual_mode = 0U;
    manual_duty = 0U;
    heater_target_duty = 0U;
    Outputs_ForceOff();
    Oven_SetState(OVEN_START);
    Sync_MarkStateFrom(active_command_source);
  }
  else if (strcmp(cmd, "CAL") == 0)
  {
    if (mpu_ok == 0U)
    {
      mpu_ok = MPU6050_Init();
    }
    if (mpu_ok != 0U)
    {
      mpu_ok = MPU6050_CalibrateZero();
    }
    Sync_MarkStateFrom(active_command_source);
  }
  else if (strcmp(cmd, "EMG=1") == 0)
  {
    emergency_latched = 1U;
    emergency_auto = 0U;
    heater_target_duty = 0U;
    Outputs_ForceOff();
    Oven_SetStateFrom(OVEN_ERROR, SRC_SAFETY);
  }
  else if (strcmp(cmd, "EMG=0") == 0)
  {
    int32_t temp_now = DHT11_ReadTempX10();
    if ((temp_now >= TEMP_CUTOFF_X10) && (temp_now >= 0L))
    {
      command_error_reason = "UNSAFE_TEMPERATURE";
      valid = 0U;
    }
    else if ((oven_data.door_is_open != 0U) &&
             (cfg.safety_return_to_start != 0UL) &&
             (Door_SafetyCanTrigger(oven_state) != 0U))
    {
      command_error_reason = "DOOR_OPEN";
      valid = 0U;
    }
    else
    {
      Oven_ClearEmergency(temp_now);
    }
  }
  else if ((strcmp(cmd, "PLUS") == 0) || (strcmp(cmd, "BTN=PLUS") == 0))
  {
    Config_Adjust(1);
  }
  else if ((strcmp(cmd, "MINUS") == 0) || (strcmp(cmd, "BTN=MINUS") == 0))
  {
    Config_Adjust(-1);
  }
  else if ((strcmp(cmd, "NEXTCFG") == 0) || (strcmp(cmd, "BTN=NEXTCFG") == 0))
  {
    Config_NextMenu();
  }
  else if ((strcmp(cmd, "BACK") == 0) || (strcmp(cmd, "BTN=BACK") == 0))
  {
    Config_Back();
  }
  else if (strcmp(cmd, "CONFIRM") == 0)
  {
    if (emergency_latched != 0U)
    {
      command_error_reason = "EMERGENCY_ACTIVE";
      valid = 0U;
    }
    else
    {
      Oven_Start();
    }
  }
  else if ((strcmp(cmd, "ENTER") == 0) || (strcmp(cmd, "BTN=ENTER") == 0))
  {
    if (oven_state == OVEN_ERROR)
    {
      Oven_ClearEmergency(DHT11_ReadTempX10());
    }
    else if ((oven_state == OVEN_START) || (oven_state == OVEN_CONFIGURATION) ||
             (oven_state == OVEN_START_CONFIRMATION))
    {
      Config_Enter();
    }
    else if ((oven_state == OVEN_COOLING) || (oven_state == OVEN_FINISHED))
    {
      Oven_SetState(OVEN_START);
    }
    else if (oven_state == OVEN_MANUAL_MODE)
    {
      Oven_Stop();
    }
    else
    {
      Oven_Stop();
    }
  }
  else if (strncmp(cmd, "PROFILE=", 8) == 0)
  {
    const char *profile = &cmd[8];
    if ((strcmp(profile, "ESTANHO") == 0) || (strcmp(profile, "TIN") == 0) ||
        (strcmp(profile, "0") == 0))
    {
      Profile_ApplyFrom(PROFILE_TIN, active_command_source);
    }
    else if ((strcmp(profile, "LATAO") == 0) || (strcmp(profile, "BRASS") == 0) ||
             (strcmp(profile, "1") == 0))
    {
      Profile_ApplyFrom(PROFILE_BRASS, active_command_source);
    }
    else if ((strcmp(profile, "ALUMINIO") == 0) || (strcmp(profile, "ALUMINUM") == 0) ||
             (strcmp(profile, "2") == 0))
    {
      Profile_ApplyFrom(PROFILE_ALUMINUM, active_command_source);
    }
    else if ((strcmp(profile, "MANUAL") == 0) || (strcmp(profile, "PERSONALIZADO") == 0) ||
             (strcmp(profile, "CUSTOM") == 0) || (strcmp(profile, "3") == 0))
    {
      Profile_ApplyFrom(PROFILE_MANUAL, active_command_source);
    }
    else
    {
      valid = 0U;
    }
  }
  else if (strncmp(cmd, "STEP=", 5) == 0)
  {
    value = strtol(&cmd[5], NULL, 10);
    config_step = (ConfigStep_t)ClampLong((int32_t)value, 0L, (int32_t)CONFIG_STEP_TOTAL - 1L);
    Oven_SetState(OVEN_CONFIGURATION);
    Sync_MarkStateFrom(active_command_source);
  }
  else if (strncmp(cmd, "TEMP=", 5) == 0)
  {
    value = strtol(&cmd[5], NULL, 10);
    cfg.setpoint_x10 = ClampLong((int32_t)value, 0L, 1200L);
    Config_MarkDirtyFrom(active_command_source);
  }
  else if (strncmp(cmd, "SET10,", 6) == 0)
  {
    value = strtol(&cmd[6], NULL, 10);
    cfg.setpoint_x10 = ClampLong((int32_t)value, 0L, 1200L);
    Config_MarkDirtyFrom(active_command_source);
  }
  else if (strncmp(cmd, "SET,", 4) == 0)
  {
    value = strtol(&cmd[4], NULL, 10);
    cfg.setpoint_x10 = ClampLong((int32_t)(value * 10L), 0L, 1200L);
    Config_MarkDirtyFrom(active_command_source);
  }
  else if (strncmp(cmd, "SET=", 4) == 0)
  {
    value = strtol(&cmd[4], NULL, 10);
    cfg.setpoint_x10 = ClampLong((int32_t)value, 0L, 1200L);
    Config_MarkDirtyFrom(active_command_source);
  }
  else if ((strncmp(cmd, "HEATTIME=", 9) == 0) || (strncmp(cmd, "HEAT=", 5) == 0))
  {
    const char *value_ptr = (cmd[4] == '=') ? &cmd[5] : &cmd[9];
    value = strtol(value_ptr, NULL, 10);
    cfg.heating_seconds = (uint32_t)ClampLong((int32_t)value, 5L, 3600L);
    Config_MarkDirtyFrom(active_command_source);
  }
  else if ((strncmp(cmd, "KP=", 3) == 0) || (strncmp(cmd, "GAIN=", 5) == 0))
  {
    value = strtol((cmd[1] == 'P') ? &cmd[3] : &cmd[5], NULL, 10);
    cfg.kp_x100 = ClampLong((int32_t)value, 0L, 1000L);
    Config_MarkDirtyFrom(active_command_source);
  }
  else if (strncmp(cmd, "ANGLE=", 6) == 0)
  {
    value = strtol(&cmd[6], NULL, 10);
    cfg.angle_limit_x10 = ClampLong((int32_t)value, 20L, 900L);
    Config_MarkDirtyFrom(active_command_source);
  }
  else if (strncmp(cmd, "SOAK=", 5) == 0)
  {
    value = strtol(&cmd[5], NULL, 10);
    if (value < 5L)
    {
      value = 5L;
    }
    else if (value > 3600L)
    {
      value = 3600L;
    }
    cfg.soak_seconds = (uint32_t)value;
    Config_MarkDirtyFrom(active_command_source);
  }
  else if ((strncmp(cmd, "SAFETY,", 7) == 0) || (strncmp(cmd, "SAFETY=", 7) == 0))
  {
    value = strtol(&cmd[7], NULL, 10);
    cfg.safety_return_to_start = (value != 0L) ? 1UL : 0UL;
    Config_MarkDirtyFrom(active_command_source);
  }
  else if (strncmp(cmd, "MANUAL=", 7) == 0)
  {
    value = strtol(&cmd[7], NULL, 10);
    value = ClampLong((int32_t)value, 0L, 100L);
    if (value == 0L)
    {
      manual_mode = 0U;
      manual_duty = 0U;
      heater_target_duty = 0U;
      Outputs_ForceOff();
      Sync_MarkStateFrom(active_command_source);
    }
    else if (emergency_latched == 0U)
    {
      Oven_SetState(OVEN_MANUAL_MODE);
      manual_mode = 1U;
      manual_duty = (uint8_t)value;
      Outputs_SetPower((uint8_t)value);
      Sync_MarkStateFrom(active_command_source);
    }
    else
    {
      command_error_reason = "EMERGENCY_ACTIVE";
      valid = 0U;
    }
  }
  else if (strcmp(cmd, "SAVE") == 0)
  {
    if (Config_Save() != 0U)
    {
      cfg_dirty = 0U;
    }
  }
  else if (strncmp(cmd, "STATE=", 6) == 0)
  {
    valid = Command_SetStateByName(&cmd[6]);
  }
  else
  {
    valid = 0U;
  }

  if (valid != 0U)
  {
    Telemetry_ForceUpdate();
  }

  return valid;
}

static void UART_PollCommands(void)
{
  uint8_t byte;

  while (uart_rx_tail != uart_rx_head)
  {
    byte = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % UART_RX_BUFFER_SIZE);

    if ((byte == '\n') || (byte == '\r'))
    {
      if (uart_rx_index > 0U)
      {
        uint8_t ok;
        uart_rx_line[uart_rx_index] = '\0';
        ok = Command_ProcessFrom((const char *)uart_rx_line, SRC_WEB);
        if (ok != 0U)
        {
          UART_SendAck((const char *)uart_rx_line);
        }
        else
        {
          UART_SendError(command_error_reason);
        }
        uart_rx_index = 0U;
      }
    }
    else if (uart_rx_index < (sizeof(uart_rx_line) - 1U))
    {
      uart_rx_line[uart_rx_index++] = byte;
    }
    else
    {
      uart_rx_index = 0U;
    }
  }
}

static void UART_StartReceive(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next_head = (uint16_t)((uart_rx_head + 1U) % UART_RX_BUFFER_SIZE);

    if (next_head != uart_rx_tail)
    {
      uart_rx_buffer[uart_rx_head] = uart_rx_byte;
      uart_rx_head = next_head;
    }

    (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    UART_StartReceive();
  }
}

static void UART_SendAck(const char *cmd)
{
  char tx[32];
  char name[18];
  uint8_t i = 0U;
  int len;

  if (cmd == NULL)
  {
    cmd = "OK";
  }

  while ((cmd[i] != '\0') && (cmd[i] != '=') && (cmd[i] != ',') && (i < (sizeof(name) - 1U)))
  {
    name[i] = cmd[i];
    i++;
  }
  name[i] = '\0';

  if (i == 0U)
  {
    snprintf(name, sizeof(name), "OK");
  }

  len = snprintf(tx, sizeof(tx), "ACK,%s\n", name);
  if (len > 0)
  {
    if (len >= (int)sizeof(tx))
    {
      len = (int)sizeof(tx) - 1;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)len, 50U);
  }
}

static void UART_SendError(const char *reason)
{
  char tx[48];
  int len = snprintf(tx, sizeof(tx), "ERR,%s\n", (reason != NULL) ? reason : "ERROR");

  if (len > 0)
  {
    if (len >= (int)sizeof(tx))
    {
      len = (int)sizeof(tx) - 1;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)len, 50U);
  }
}

static void UART_SendStatus(void)
{
  char tx[1400];
  int len = snprintf(tx, sizeof(tx),
                     "DATA,TEMP10=%ld,SP10=%ld,HEATER=%u,POWER=%u,STATE=%s,DOOR10=%ld,DOORY10=%ld,DOOROPEN=%u,SAFETY=%u,MODE=%s,temp_x10=%ld,angx_x10=%ld,angy_x10=%ld,mpu=%u,cal=%u,emg=%u,auto=%u,state=%s,profile=%s,profile_id=%lu,step=%s,step_id=%u,set_x10=%ld,heat_time=%lu,soak=%lu,duty=%u,heat=%u,kp=%ld,angle=%ld,manual=%u,progress=%u,sec=%lu,menu=%s,dirty=%u,led_ready=%u,led_heat=%u,led_hold=%u,led_done=%u,led_emg=%u,led_pwm=%u,led_life=%u,CFG_REV=%lu,CFG_SRC=%s,STATE_REV=%lu,STATE_SRC=%s,SYNC_REV=%lu,SYNC_SRC=%s,PWM=%u,LED_PWM_LEVEL=%u\n",
                     oven_data.temperature_x10,
                     oven_data.setpoint_x10,
                     (unsigned int)oven_data.heater_relay_on,
                     (unsigned int)oven_data.heater_output_percent,
                     Oven_StateName(oven_data.current_state),
                     oven_data.door_angle_x10,
                     oven_data.door_angle_y10,
                     (unsigned int)oven_data.door_is_open,
                     (unsigned int)oven_data.safety_return_to_start,
                     Oven_ModeName(oven_data.current_state),
                     oven_data.temperature_x10,
                     oven_data.door_angle_x10,
                     oven_data.door_angle_y10,
                     (unsigned int)oven_data.mpu_ok,
                     (unsigned int)oven_data.mpu_calibrated,
                     (unsigned int)oven_data.emergency_active,
                     (unsigned int)oven_data.emergency_auto,
                     Oven_StateName(oven_data.current_state),
                     Profile_Name((MaterialProfile_t)cfg.profile),
                     (unsigned long)cfg.profile,
                     Config_StepName(config_step),
                     (unsigned int)config_step,
                     oven_data.setpoint_x10,
                     (unsigned long)cfg.heating_seconds,
                     (unsigned long)cfg.soak_seconds,
                     (unsigned int)oven_data.heater_output_percent,
                     (unsigned int)oven_data.heater_relay_on,
                     cfg.kp_x100,
                     cfg.angle_limit_x10,
                     (unsigned int)oven_data.manual_mode,
                     (unsigned int)oven_data.progress_percent,
                     (unsigned long)oven_data.seconds,
                     Config_MenuName(menu_item),
                     (unsigned int)cfg_dirty,
                     (unsigned int)oven_data.led_ready,
                     (unsigned int)oven_data.led_heating,
                     (unsigned int)oven_data.led_holding,
                     (unsigned int)oven_data.led_done,
                     (unsigned int)oven_data.led_emergency,
                     (unsigned int)oven_data.led_pwm_mirror,
                     (unsigned int)oven_data.led_heartbeat,
                     (unsigned long)cfg_rev,
                     Sync_SourceName(cfg_source),
                     (unsigned long)state_rev,
                     Sync_SourceName(state_source),
                     (unsigned long)sync_rev,
                     Sync_SourceName(sync_source),
                     (unsigned int)heater_duty,
                     (unsigned int)oven_data.led_pwm_level);

  if (len > 0)
  {
    if (len >= (int)sizeof(tx))
    {
      len = (int)sizeof(tx) - 1;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)len, 50U);
  }
}

static void UART_SendLog(int32_t temp_x10)
{
  char tx[96];
  int len = snprintf(tx, sizeof(tx),
                     "LOG,sec=%lu,temp_x10=%ld,state=%s,duty=%u\n",
                     (unsigned long)seconds_on,
                     temp_x10,
                     Oven_StateName(oven_state),
                     (unsigned int)heater_duty);

  if (len > 0)
  {
    if (len >= (int)sizeof(tx))
    {
      len = (int)sizeof(tx) - 1;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)len, 50U);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  UART_StartReceive();
  MX_TIM3_Init();
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  Outputs_ForceOff();
#ifdef HAL_IWDG_MODULE_ENABLED
  MX_IWDG_Init();
#endif

  DWT_DelayInit();
  Config_Load();

  oled_ok = SSD1306_Init(&hi2c1);
  mpu_ok = MPU6050_Init();
  if (mpu_ok != 0U)
  {
    mpu_ok = MPU6050_CalibrateZero();
  }

  state_enter_ms = HAL_GetTick();
  last_control_ms = state_enter_ms;
  last_uart_ms = state_enter_ms;
  last_log_ms = state_enter_ms;
  last_soft_ms = state_enter_ms;
  OvenData_Update(-1L, 0L, 0L, 0U, state_enter_ms);

  while (1)
  {
    uint32_t now = HAL_GetTick();
    static int32_t temp_x10 = -1L;
    static int32_t ang_x10 = 0L;
    static int32_t ang_y10 = 0L;
    static uint8_t mpu_read_ok = 0U;

#ifdef HAL_IWDG_MODULE_ENABLED
    HAL_IWDG_Refresh(&hiwdg);
#endif

    UART_PollCommands();
    Buttons_Poll(now);
    Config_AutoSave(now);

    if ((now - last_led_ms) >= 100U)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      LEDs_Update(now);
    }

    if ((now - last_control_ms) >= CONTROL_PERIOD_MS)
    {
      int16_t ax = 0;
      int16_t ay = 0;
      int16_t az = 0;

      last_control_ms = now;
      seconds_on = now / 1000UL;
      temp_x10 = DHT11_ReadTempX10();

      if (oled_ok == 0U)
      {
        oled_ok = SSD1306_Init(&hi2c1);
      }

      if (mpu_ok == 0U)
      {
        mpu_ok = MPU6050_Init();
      }

      mpu_read_ok = MPU6050_ReadAccel(&ax, &ay, &az);
      if (mpu_read_ok == 0U)
      {
        mpu_ok = 0U;
      }
      else
      {
        ang_x10 = MPU6050_ApproxAngleDegX10(ax, accel_x_zero);
        ang_y10 = MPU6050_ApproxAngleDegX10(ay, accel_y_zero);
      }

      Oven_UpdateState(temp_x10, ang_x10, ang_y10, mpu_read_ok);
      Heater_UpdateDuty(temp_x10, now);
      OvenData_Update(temp_x10, ang_x10, ang_y10, mpu_read_ok, now);

      if (oled_ok != 0U)
      {
        OLED_ShowScreen(&oven_data);
      }
    }

    if ((telemetry_force_update != 0U) || ((now - last_uart_ms) >= UART_PERIOD_MS))
    {
      last_uart_ms = now;
      OvenData_Update(temp_x10, ang_x10, ang_y10, mpu_read_ok, now);
      UART_SendStatus();
      telemetry_force_update = 0U;
    }

    if ((now - last_log_ms) >= LOG_PERIOD_MS)
    {
      last_log_ms = now;
      UART_SendLog(temp_x10);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (uint32_t)((HAL_RCC_GetPCLK1Freq() / 1000000UL) - 1UL);
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (1000000UL / HEATER_PWM_FREQ_HZ) - 1UL;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

#ifdef HAL_IWDG_MODULE_ENABLED
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_EMERGENCY_GPIO_Port, LED_EMERGENCY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HEATING_GPIO_Port, LED_HEATING_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HOLDING_GPIO_Port, LED_HOLDING_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_DONE_GPIO_Port, LED_DONE_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13 | LED_EMERGENCY_Pin | LED_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_HEATING_Pin | LED_HOLDING_Pin | LED_DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HEATER_PWM_Pin | LED_PWM_MIRROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BTN_PLUS_Pin | BTN_MINUS_Pin | BTN_ENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    for (volatile uint32_t i = 0U; i < 80000U; i++)
    {
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
