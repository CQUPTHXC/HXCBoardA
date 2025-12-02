#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "common_inc.h"
#include "BspDevice.h"

// 芯片主频 (MHz)，用于根据频率推算 TIM 寄存器
#define CHIP_FREQ_MHZ 180.0f

#ifdef __cplusplus
extern "C" {
#endif

// 供 HAL 定时器中断回调转发使用，不应由用户直接调用
void Timer_Callback_Trampoline(void *_TimerHandele);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// Timer 封装了对硬件 TIM 的管理：申请句柄、配置频率、启动/停止以及回调注册
// - 用户需要在构造时传入合法的 `DEVICE_TIMER_*` 枚举，否则对象将保持未初始化状态
// - Init/Start/Stop/SetCallback 均返回 BspResult，调用后务必检查是否成功以捕获资源冲突或参数错误
// - 成功 Start 之后，占用了对应的 TIM，调用 Stop 释放；未 Stop 直接析构将保持占用状态
class Timer
{
private:
  TIM_HandleTypeDef* htim = nullptr;
  BspDevice_t deviceID = DEVICE_NONE;
  uint32_t freq = 100; // 频率 (Hz)
  uint16_t PSC = 180 - 1; // 预分频器
  uint32_t ARR = 10000 - 1; // 自动重装载值

  Callback_t selfTimerCallback = nullptr;

  void CalcRegister(uint32_t _freq);

public:
  // 构造时仅接受枚举范围内的 TIM 设备；无效设备会导致对象处于不可用状态
  explicit Timer(BspDevice_t _deviceID)
  {
    auto isValidDevice = (_deviceID >= DEVICE_TIMER_START && _deviceID < DEVICE_TIMER_END);
    if (isValidDevice)
    {
      deviceID = _deviceID;
    }
    else
    {
      deviceID = DEVICE_NONE;
      htim = nullptr;
      return;
    } 
  }

  // 完成句柄绑定并配置频率；频率过高或句柄未注册会返回失败
  BspResult<bool> Init(uint32_t _freqHz);
  // 启动定时器并标记资源占用；重复 Start 会返回占用错误
  BspResult<bool> Start();
  // 停止定时器并释放占用；仅在成功 Start 后调用
  BspResult<bool> Stop();
  // 注册用户回调，在中断中由框架代为触发；若传入 nullptr 等同于取消回调
  BspResult<bool> SetCallback(Callback_t _timerCallback);
  // 供内部/测试手动触发回调；要求先通过 SetCallback 设置函数
  void InvokeCallback(); // 添加一个公共的调用函数
  // 获取当前配置（频率、PSC、ARR 等）字符串，便于调试
  const char* GetInfo() const;
};
#endif // __cplusplus



#endif // __BSP_TIMER_H__