#include "Y_foc.h"

YFoc* YFoc::instances[YFoc::MAX_MOTOR_INSTANCES] = {nullptr};
uint8_t YFoc::instanceCount = 0;

YFoc::YFoc(USE_CanBus _bus, MotorCanId _baseId)
 : bus(_bus), 
  baseId(_baseId),
  feedbackId(static_cast<uint32_t>(baseId) | static_cast<uint32_t>(MotorCanExtId::STATUS_FEEDBACK))
{
  // 注册接收回调

  
  memset(&currentMitCmd, 0, sizeof(currentMitCmd));
  memset(txBuffer, 0, sizeof(txBuffer));
  memset(&status, 0, sizeof(status));
  
  // 将实例添加到静态数组
  if (instanceCount < MAX_MOTOR_INSTANCES)
  {
    instances[instanceCount++] = this;
  }

  auto& canMgr = CanManager::GetInstance();
  MW_Status res = canMgr.AskResource(bus, Can::BAUD_1M, Can::MODE_NORMAL);
}

YFoc::~YFoc()
{
  // 从静态数组中移除实例
  for (uint8_t i = 0; i < instanceCount; ++i)
  {
    if (instances[i] == this)
    {
      auto& canMgr = CanManager::GetInstance();
      canMgr.UnSubscribe(bus, feedbackId, YFoc::CanRxCallback);
      instances[i] = instances[instanceCount - 1];
      instances[instanceCount - 1] = nullptr;
      --instanceCount;
      break;
    }
  }
}


bool YFoc::Init()
{
  auto& canMgr = CanManager::GetInstance();

  MW_Status res = canMgr.StartResource(bus);
  if (res != MW_Status::SUCCESS) 
    return false;

  res = canMgr.Subscribe(bus, feedbackId, YFoc::CanRxCallback);
  return (res == MW_Status::SUCCESS);
}



void YFoc::SendMitCmd(const MITCmd_t& cmd)
{
  // 将浮点数转换为无符号整数
  uint32_t posInt = FloatToUint(cmd.position, MIT_POS_MAX, MIT_POS_MIN, MIT_POS_BITS);
  uint32_t velInt = FloatToUint(cmd.velocity, MIT_VEL_MAX, MIT_VEL_MIN, MIT_VEL_BITS);
  uint32_t torqueInt = FloatToUint(cmd.torque, MIT_TORQUE_MAX, MIT_TORQUE_MIN, MIT_TORQUE_BITS);
  uint32_t kpInt = FloatToUint(cmd.kp, MIT_KP_MAX, MIT_KP_MIN, MIT_KP_BITS);
  uint32_t kdInt = FloatToUint(cmd.kd, MIT_KD_MAX, MIT_KD_MIN, MIT_KD_BITS);

  uint8_t txBuffer[8];
  // 打包数据到 txBuffer
  txBuffer[0] = (posInt >> 4) & 0xFF;
  txBuffer[1] = ((posInt & 0x0F) << 4) | ((velInt >> 8) & 0x0F);
  txBuffer[2] = velInt & 0xFF;
  txBuffer[3] = (torqueInt >> 4) & 0xFF;
  txBuffer[4] = ((torqueInt & 0x0F) << 4) | ((kpInt >> 8) & 0x0F);
  txBuffer[5] = kpInt & 0xFF;
  txBuffer[6] = (kdInt >> 4) & 0xFF;
  txBuffer[7] = (kdInt & 0x0F) << 4;

  cmdId = static_cast<uint32_t>(baseId) | static_cast<uint32_t>(MotorCanExtId::MIT_CTRL);

  CanMessage msg;
  msg.id = cmdId;
  msg.isExtended = false;
  msg.isRemote = false;
  msg.len = 8;
  memcpy(msg.data, txBuffer, 8);

  CanManager::GetInstance().sendMessage(bus, msg);

}

bool YFoc::CheckTimeout(uint32_t timeoutMs)
{
  uint32_t currentTime = HAL_GetTick();
  if ((currentTime - status.lastRxTime) > timeoutMs)
  {
    status.isOnline = false;
    return true;
  }
  return false;
}

uint32_t YFoc::FloatToUint(float x, float x_min, float x_max, uint8_t bits)
{
  if (x_max <= x_min) 
    return 0;
  const uint32_t maxInt = (1u << bits) - 1u;

  float span = x_max - x_min;
  float normalized = (x - x_min) / span; // in [0,1]

  if (normalized < 0.0f) 
    normalized = 0.0f;
  if (normalized > 1.0f) 
    normalized = 1.0f;
  // 四舍五入到 nearest
  uint32_t val = (uint32_t)(normalized * (float)maxInt + 0.5f);

  if (val > maxInt) 
    val = maxInt;

  return val;
}

float YFoc::UintToFloat(uint32_t val, float x_min, float x_max, uint8_t bits)
{
  const uint32_t maxInt = (1u << bits) - 1u;
  if (maxInt == 0) 
    return x_min;
  float span = x_max - x_min;
  return ((float)val) * span / (float)maxInt + x_min;
}


void YFoc::ProcessFeedback(uint32_t canId, uint8_t* data, uint8_t len)
{
  float posRaw;
  uint16_t velRaw = (uint16_t)data[4] << 8 | data[5];
  uint16_t torqueRaw = (uint16_t)data[6] << 8 | data[7];

  memcpy(&posRaw, &data[0], sizeof(float));

  status.position = posRaw;
  status.velocity = UintToFloat(velRaw, -200, 200, 16);
  status.torque = UintToFloat(torqueRaw, -0.28, 0.28, 16);

  status.isOnline = true;
  status.lastRxTime = HAL_GetTick();

}

void YFoc::CanRxCallback(uint32_t canId, uint8_t* data, uint8_t len)
{
  for (uint8_t i = 0; i < instanceCount; i++)
  {
    if (instances[i]->feedbackId == canId)
    {
      instances[i]->ProcessFeedback(canId, data, len);
      break;
    }
  }
}

