#ifndef __Y_FOC_H__
#define __Y_FOC_H__

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#ifdef __cplusplus


#include "common_inc.h"
#include "BspCan.h"
#include "B2MW_CANManager.hpp"



enum MotorCanId : uint32_t
{
  M1_ID = 0x100, 
  M2_ID = 0x200, 
  M3_ID = 0x300, 
  M4_ID = 0x400,
};

enum MotorCanExtId : uint32_t
{
  TORQUE_CTRL     = 0x1,
  VELOCITY_CTRL   = 0x2,
  POSITION_CTRL   = 0x3,
  MIT_CTRL        = 0x4,
  STATUS_FEEDBACK = 0x5,
};

struct MITCmd_t
{
  float position;
  float velocity;
  float torque;
  float kp;
  float kd;
};

struct MotorStatus
{
  float position;      // 位置 (rad)
  float velocity;      // 速度 (rad/s)
  float torque;        // 力矩 (Nm)
  uint8_t statusCode;  // 错误码
  bool isOnline;       // 在线状态
  uint32_t lastRxTime; // 最后接收时间戳(ms)
};

class YFoc
{
private:

  static constexpr float MIT_POS_MAX    = _PI;  // 位置最大值限制(rad)
  static constexpr float MIT_POS_MIN    = -_PI; // 位置最小值限制(rad)
  static constexpr float MIT_VEL_MAX    = 100.0f;  // 速度最大值限制(rad/s)
  static constexpr float MIT_VEL_MIN    = -100.0f;  // 速度最小值限制(rad/s)
  static constexpr float MIT_TORQUE_MAX = 0.14f; // 力矩最大值限制(Nm)
  static constexpr float MIT_TORQUE_MIN = -0.14f; // 力矩最小值限制(Nm)
  static constexpr float MIT_KP_MAX     = 1.0f; // kp 最大值限制
  static constexpr float MIT_KP_MIN     = 0.0f;    // kp 最小
  static constexpr float MIT_KD_MAX     = 0.1f; // kd 最大值限制
  static constexpr float MIT_KD_MIN     = 0.0f;    // kd 最小

  static constexpr uint8_t MIT_POS_BITS     = 16;   // 位置位数
  static constexpr uint8_t MIT_VEL_BITS     = 12;   // 速度位数
  static constexpr uint8_t MIT_TORQUE_BITS  = 12; // 力矩位数
  static constexpr uint8_t MIT_KP_BITS      = 12;     // kp 位数
  static constexpr uint8_t MIT_KD_BITS      = 12;     // kd 位数

  static constexpr uint8_t MAX_MOTOR_INSTANCES = 8;
  static YFoc* instances[MAX_MOTOR_INSTANCES];
  static uint8_t instanceCount;
  
  MotorCanId baseId;      // 基础ID（M1_ID, M2_ID, ...）
  uint32_t cmdId;         // 指令ID（baseId | 0x04）
  uint32_t feedbackId;    // 反馈ID（baseId | 0x04）
  
  MITCmd_t currentMitCmd;    // 当前指令
  MotorStatus status;     // 电机状态
  uint8_t txBuffer[8] = {0};    // 发送缓冲

  USE_CanBus bus = USE_CAN1;

  void ProcessFeedback(uint32_t canId, uint8_t* data, uint8_t len);
  static void CanRxCallback(uint32_t canId, uint8_t* data, uint8_t len);



  static uint32_t FloatToUint(float x, float x_min, float x_max, uint8_t bits);
  static float UintToFloat(uint32_t val, float x_min, float x_max, uint8_t bits);

public:


  explicit YFoc(USE_CanBus _bus, MotorCanId _baseId);
  ~YFoc();

  bool Init();
  void SendMitCmd(const MITCmd_t& cmd);

  const MotorStatus& GetStatus() const { return status; }

  bool CheckTimeout(uint32_t timeoutMs);


};


#endif // __cplusplus

#endif // !__Y_FOC_H__
