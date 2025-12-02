#include "BspPwm.h"
#include <cstdint>

extern "C"
{
}

static const char* TimerInstanceName(const TIM_TypeDef* instance)
{
  if (instance == TIM1) return "TIM1";
  if (instance == TIM2) return "TIM2";
  if (instance == TIM3) return "TIM3";
  if (instance == TIM4) return "TIM4";
  if (instance == TIM5) return "TIM5";
  if (instance == TIM6) return "TIM6";
  if (instance == TIM7) return "TIM7";
  if (instance == TIM8) return "TIM8";
  if (instance == TIM9) return "TIM9";
  if (instance == TIM10) return "TIM10";
  if (instance == TIM11) return "TIM11";
  if (instance == TIM12) return "TIM12";
  if (instance == TIM13) return "TIM13";
  if (instance == TIM14) return "TIM14";
  return "UNKNOWN";
}

static const char* TimStateToString(HAL_TIM_StateTypeDef state)
{
  switch (state)
  {
  case HAL_TIM_STATE_RESET:   return "RESET";
  case HAL_TIM_STATE_READY:   return "READY";
  case HAL_TIM_STATE_BUSY:    return "BUSY";
  case HAL_TIM_STATE_TIMEOUT: return "TIMEOUT";
  case HAL_TIM_STATE_ERROR:   return "ERROR";
  default:                    return "UNKNOWN";
  }
}

static const char* CounterModeToString(uint32_t mode)
{
  switch (mode)
  {
  case TIM_COUNTERMODE_UP:             return "UP";
  case TIM_COUNTERMODE_DOWN:           return "DOWN";
  case TIM_COUNTERMODE_CENTERALIGNED1: return "CENTER1";
  case TIM_COUNTERMODE_CENTERALIGNED2: return "CENTER2";
  case TIM_COUNTERMODE_CENTERALIGNED3: return "CENTER3";
  default:                             return "UNKNOWN";
  }
}

/**
 * @brief PWM初始化函数
 * @param _freqHz PWM频率 (Hz)
 * @return BspResult<bool> 操作结果
 * @note  - 获取设备句柄
 *        - 计算并设置PSC和ARR
 *        - 不启动PWM，需要手动调用Start()
 */
BspResult<bool> Pwm::Init(uint32_t _freqHz)
{
  // 频率范围限制
  if (_freqHz < 1) _freqHz = 1;
  else if (_freqHz > 10000000) _freqHz = 10000000;

  BSP_CHECK(deviceID >= DEVICE_PWM_START && deviceID < DEVICE_PWM_END, BspError::InvalidDevice, bool);

  auto handleResult = Bsp_GetDeviceHandle(deviceID);
  BSP_CHECK(handleResult.ok() && handleResult.value != nullptr, handleResult.error, bool);
  htim = static_cast<TIM_HandleTypeDef*>(handleResult.value);

  BSP_CHECK(htim->Instance != nullptr, BspError::InvalidDevice, bool);

  freq = _freqHz;
  CalcRegister(_freqHz);
  
  __HAL_TIM_SET_PRESCALER(htim, PSC);
  __HAL_TIM_SET_AUTORELOAD(htim, ARR);
  __HAL_TIM_SET_COUNTER(htim, 0);

  return BspResult<bool>::success(true);
}





/**
 * @brief  根据期望的频率计算PSC和ARR寄存器值
 * @param  _freq 期望的频率 (Hz)
 * @note   这是一个内部辅助函数，用于寻找最优的PSC和ARR组合
 */
void Pwm::CalcRegister(uint32_t _freq)
{
  bool isAPB2 = (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 || htim->Instance == TIM10 || htim->Instance == TIM11);
  
  uint32_t apbFreq = isAPB2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();

  RCC_ClkInitTypeDef clk;
  uint32_t flashLatency;
  HAL_RCC_GetClockConfig(&clk, &flashLatency);

  bool apbDivGt1 = isAPB2 ? (clk.APB2CLKDivider != RCC_HCLK_DIV1) : (clk.APB1CLKDivider != RCC_HCLK_DIV1);

  uint32_t timClk = apbFreq * (apbDivGt1 ? 2 : 1); // 定时器时钟频率

  uint32_t bestPSC = 0, bestARR = 0;
  for (uint32_t psc = 0; psc <= 0xFFFF; ++psc) 
  {
    uint32_t arrPlus1 = timClk / (_freq * (psc + 1));
    if (arrPlus1 <= 1) 
      break;

    uint32_t arr = arrPlus1 - 1;
    if (arr <= 0xFFFF) 
    {
      bestARR = arr;
      bestPSC = psc;
      break;
    }
  }
  PSC = (uint16_t)bestPSC;
  ARR = (uint32_t)bestARR;
  freq = timClk / ((PSC + 1) * (ARR + 1)); // 实际频率
}

/**
 * @brief  启动PWM输出
 * @param  channel PWM通道
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Pwm::Start(PwmChannel_t channel)
{
  BSP_CHECK(htim != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(deviceID >= DEVICE_PWM_START && deviceID < DEVICE_PWM_END, BspError::InvalidDevice, bool);

  HAL_StatusTypeDef status = HAL_OK;

  if (channel == CHANNEL_ALL || channel == CHANNEL_1)
  {
    status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }
  if (channel == CHANNEL_ALL || channel == CHANNEL_2)
  {
    status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }
  if (channel == CHANNEL_ALL || channel == CHANNEL_3)
  {
    status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }
  if (channel == CHANNEL_ALL || channel == CHANNEL_4)
  {
    status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }

  auto startResult = Bsp_StartDevice(deviceID);
  BSP_CHECK(startResult.ok(), startResult.error, bool);

  return BspResult<bool>::success(true);
}

/**
 * @brief  停止PWM输出
 * @param  channel PWM通道
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Pwm::Stop(PwmChannel_t channel)
{
  BSP_CHECK(htim != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(deviceID >= DEVICE_PWM_START && deviceID < DEVICE_PWM_END, BspError::InvalidDevice, bool);

  HAL_StatusTypeDef status = HAL_OK;

  if (channel == CHANNEL_ALL || channel == CHANNEL_1)
  {
    status = HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }
  if (channel == CHANNEL_ALL || channel == CHANNEL_2)
  {
    status = HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }
  if (channel == CHANNEL_ALL || channel == CHANNEL_3)
  {
    status = HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }
  if (channel == CHANNEL_ALL || channel == CHANNEL_4)
  {
    status = HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_4);
    if (status != HAL_OK)
      return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }

  auto stopResult = Bsp_StopDevice(deviceID);
  BSP_CHECK(stopResult.ok(), stopResult.error, bool);

  return BspResult<bool>::success(true);
}

/**
 * @brief  设置PWM频率
 * @param  freqHz 频率 (Hz)
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Pwm::SetFrequency(uint32_t freqHz)
{
  BSP_CHECK(htim != nullptr, BspError::NullHandle, bool);
  
  if (freqHz < 1)
    freqHz = 1;
  else if (freqHz > 10000000)
    freqHz = 10000000;

  freq = freqHz;
  CalcRegister(freqHz);
  __HAL_TIM_SET_PRESCALER(htim, PSC);
  __HAL_TIM_SET_AUTORELOAD(htim, ARR);
  __HAL_TIM_SET_COUNTER(htim, 0); // 重置计数器

  return BspResult<bool>::success(true);
}

/**
 * @brief  设置占空比（使用Ticks值）
 * @param  channel PWM通道
 * @param  ticks 占空比计数值，范围0~ARR
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Pwm::SetDutyTicks(PwmChannel_t channel, uint32_t ticks)
{
  BSP_CHECK(htim != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(channel != CHANNEL_ALL, BspError::InvalidParam, bool);

  if (ticks > ARR)
    ticks = ARR;

  switch (channel)
  {
    case CHANNEL_1:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ticks);
      break;
    case CHANNEL_2:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ticks);
      break;
    case CHANNEL_3:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ticks);
      break;
    case CHANNEL_4:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, ticks);
      break;
    default:
      BSP_RETURN_FAILURE(BspError::InvalidParam, bool);
  }

  return BspResult<bool>::success(true);
}

/**
 * @brief  设置占空比（使用百分比）
 * @param  channel PWM通道
 * @param  dutyCycle 占空比，范围0.0~1.0
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Pwm::SetDutyCycle(PwmChannel_t channel, float dutyCycle)
{
  BSP_CHECK(htim != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(channel != CHANNEL_ALL, BspError::InvalidParam, bool);

  if (dutyCycle < 0.0f)
    dutyCycle = 0.0f;
  else if (dutyCycle > 1.0f)
    dutyCycle = 1.0f;

  uint32_t ticks = static_cast<uint32_t>(dutyCycle * ARR);

  switch (channel)
  {
    case CHANNEL_1:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ticks);
      break;
    case CHANNEL_2:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ticks);
      break;
    case CHANNEL_3:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ticks);
      break;
    case CHANNEL_4:
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, ticks);
      break;
    default:
      BSP_RETURN_FAILURE(BspError::InvalidParam, bool);
  }

  return BspResult<bool>::success(true);
}

/**
 * @brief  获取当前PWM频率
 * @return BspResult<uint32_t> 操作结果，成功返回频率值
 */
BspResult<uint32_t> Pwm::GetFrequency() const
{
  return BspResult<uint32_t>::success(freq);
}

/**
 * @brief  获取ARR寄存器值
 * @return BspResult<uint32_t> 操作结果，成功返回ARR值
 */
BspResult<uint32_t> Pwm::GetARR() const
{
  return BspResult<uint32_t>::success(ARR);
}

/**
 * @brief  获取PSC寄存器值
 * @return BspResult<uint16_t> 操作结果，成功返回PSC值
 */
BspResult<uint16_t> Pwm::GetPSC() const
{
  return BspResult<uint16_t>::success(PSC);
}

const char* Pwm::GetInfo()
{
  if (htim == nullptr) return "Error: Null Handle";

  static char infoBuffer[512];
  TIM_HandleTypeDef* handle = htim;
  const uint32_t compare1 = __HAL_TIM_GET_COMPARE(handle, TIM_CHANNEL_1);
  const uint32_t compare2 = __HAL_TIM_GET_COMPARE(handle, TIM_CHANNEL_2);
  const uint32_t compare3 = __HAL_TIM_GET_COMPARE(handle, TIM_CHANNEL_3);
  const uint32_t compare4 = __HAL_TIM_GET_COMPARE(handle, TIM_CHANNEL_4);
  
  const float duty1 = ARR ? (static_cast<float>(compare1) / static_cast<float>(ARR)) * 100.0f : 0.0f;
  const float duty2 = ARR ? (static_cast<float>(compare2) / static_cast<float>(ARR)) * 100.0f : 0.0f;
  const float duty3 = ARR ? (static_cast<float>(compare3) / static_cast<float>(ARR)) * 100.0f : 0.0f;
  const float duty4 = ARR ? (static_cast<float>(compare4) / static_cast<float>(ARR)) * 100.0f : 0.0f;
  
  snprintf(infoBuffer, sizeof(infoBuffer),
          "===== %s Info =====\n"
          "Device ID: %d\n"
          "TimerFreq: %u Hz\n"
          "PSC: %u\n"
          "ARR: %u\n"
          "Duty(CH1): %.1f%%\n"
          "Duty(CH2): %.1f%%\n"
          "Duty(CH3): %.1f%%\n"
          "Duty(CH4): %.1f%%\n"
          "Callbacks: %s\n"
          "=======================\n",
          TimerInstanceName(handle->Instance),
          deviceID,
          freq,
          PSC,
          ARR,
          duty1,
          duty2,
          duty3,
          duty4,
          selfTimerCallback ? "SET" : "NULL"
  );
  return infoBuffer;
}