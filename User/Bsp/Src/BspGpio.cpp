#include "BspGpio.h"
#include <cstdio>

// 静态成员变量定义
Gpio* Gpio::extiInstances[16] = {nullptr};

// 辅助函数：获取引脚编号（0-15）
static uint8_t GetPinNumber(uint16_t pin)
{
  for (uint8_t i = 0; i < 16; i++)
  {
    if (pin == (1U << i))
      return i;
  }
  return 0xFF; // 无效引脚
}

// 获取引脚索引（0-15）
uint8_t Gpio::GetPinIndex(uint16_t pin)
{
  return GetPinNumber(pin);
}

/**
 * @brief 设置GPIO输出电平
 * @param state 电平状态
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Gpio::Write(GpioState state)
{
  BSP_CHECK(port != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(pin != 0, BspError::InvalidParam, bool);

  HAL_GPIO_WritePin(port, pin, static_cast<GPIO_PinState>(state));

  return BspResult<bool>::success(true);
}

/**
 * @brief 读取GPIO输入电平
 * @return BspResult<GpioState> 电平状态
 */
BspResult<Gpio::GpioState> Gpio::Read()
{
  BSP_CHECK(port != nullptr, BspError::InvalidParam, GpioState);
  BSP_CHECK(pin != 0, BspError::InvalidParam, GpioState);

  GPIO_PinState pinState = HAL_GPIO_ReadPin(port, pin);
  GpioState state = (pinState == GPIO_PIN_SET) ? HIGH : LOW;

  return BspResult<GpioState>::success(state);
}

/**
 * @brief 翻转GPIO输出电平
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Gpio::Toggle()
{
  BSP_CHECK(port != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(pin != 0, BspError::InvalidParam, bool);

  HAL_GPIO_TogglePin(port, pin);

  return BspResult<bool>::success(true);
}

/**
 * @brief 注册外部中断回调函数
 * @param callback 中断回调函数
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Gpio::RegisterExtiCallback(GpioExtiCallback_t callback)
{
  BSP_CHECK(port != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(pin != 0, BspError::InvalidParam, bool);

  userExtiCallback = callback;

  // 保存实例以便中断回调
  uint8_t pinIndex = GetPinIndex(pin);
  if (pinIndex < 16)
  {
    extiInstances[pinIndex] = this;
  }

  return BspResult<bool>::success(true);
}

// ============================================================================
// 外部中断回调 - C接口
// ============================================================================

extern "C" {

/**
 * @brief GPIO外部中断回调跳板函数
 * @param gpioPin 触发中断的GPIO引脚
 * @note 此函数会从HAL_GPIO_EXTI_IRQHandler调用
 */
void Gpio_ExtiCallback_Trampoline(uint16_t gpioPin)
{
  uint8_t pinIndex = GetPinNumber(gpioPin);
  if (pinIndex < 16 && Gpio::extiInstances[pinIndex] != nullptr)
  {
    Gpio* instance = Gpio::extiInstances[pinIndex];
    if (instance->userExtiCallback != nullptr)
    {
      instance->userExtiCallback(gpioPin);
    }
  }
}



} // extern "C"