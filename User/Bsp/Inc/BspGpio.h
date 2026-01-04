#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "common_inc.h"
#include "BspStatus.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// GPIO 中断回调函数类型
typedef void (*GpioExtiCallback_t)(uint16_t gpioPin);

void Gpio_ExtiCallback_Trampoline(uint16_t gpioPin);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Gpio
{
private:
  GPIO_TypeDef* port = nullptr;
  uint16_t pin = 0;
  
  GpioExtiCallback_t userExtiCallback = nullptr;

  static Gpio* extiInstances[16]; // 最多16个外部中断线
  static uint8_t GetPinIndex(uint16_t pin);

public:
  /**
   * @brief GPIO电平枚举
   */
  enum GpioState : uint8_t
  {
    LOW  = GPIO_PIN_RESET,
    HIGH = GPIO_PIN_SET
  };

  /**
   * @brief 构造函数（假设CubeMX已配置好GPIO）
   * @param _port GPIO端口 (GPIOA, GPIOB, ...)
   * @param _pin GPIO引脚 (GPIO_PIN_0 ~ GPIO_PIN_15)
   */
  Gpio(GPIO_TypeDef* _port, uint16_t _pin)
    : port(_port), pin(_pin)
  {
  }

  /**
   * @brief 设置GPIO输出电平
   * @param state 电平状态 (LOW/HIGH)
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Write(GpioState state);

  /**
   * @brief 读取GPIO输入电平
   * @return BspResult<GpioState> 电平状态
   */
  BspResult<GpioState> Read();

  /**
   * @brief 翻转GPIO输出电平
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Toggle();

  /**
   * @brief 注册外部中断回调函数（假设CubeMX已配置好中断）
   * @param callback 中断回调函数
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> RegisterExtiCallback(GpioExtiCallback_t callback);

  // 友元函数，用于中断处理
  friend void Gpio_ExtiCallback_Trampoline(uint16_t gpioPin);
};

#endif // __cplusplus

#endif // __BSP_GPIO_H__
