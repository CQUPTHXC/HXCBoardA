#include "DM_Imu.h"


static DM_Imu* imuInstance = nullptr;

DM_Imu::DM_Imu(USE_CanBus _bus)
  : bus(_bus)
{
  imuInstance = this;

  auto& canMgr = CanManager::GetInstance();
  MW_Status res = canMgr.AskResource(bus, Can::BAUD_1M, Can::MODE_NORMAL);
}

DM_Imu::~DM_Imu()
{
  auto& canMgr = CanManager::GetInstance();
  canMgr.UnSubscribe(bus, canRxId, DM_Imu::CanRxCallback);
  imuInstance = nullptr;
}

bool DM_Imu::Init()
{

  auto& canMgr = CanManager::GetInstance();

  MW_Status res = canMgr.StartResource(bus);
  if (res != MW_Status::SUCCESS) 
    return false;

  res = canMgr.Subscribe(bus, canRxId, DM_Imu::CanRxCallback);
  return (res == MW_Status::SUCCESS);
}

bool DM_Imu::RequestEuler(uint8_t reg)
{
  uint8_t cmd[4] = 
  {
    (uint8_t)0x001,
    (uint8_t)(0x001 >> 8),
    reg,
    0xCC
  };
  CanMessage msg;

  msg.id = 0x6FF;
  msg.isExtended = false;
  msg.isRemote = false;
  msg.len = 4;
  memcpy(msg.data, cmd, 4);

  auto& canMgr = CanManager::GetInstance();
  MW_Status res = canMgr.sendMessage(bus, msg);
  return (res == MW_Status::SUCCESS);

}

void DM_Imu::ProcessFeedback(uint32_t canId, uint8_t* pData, uint8_t len)
{
  if (len < 8) return; // 数据长度不足

  int euler[3];
  euler[0]=pData[3]<<8|pData[2];
  euler[1]=pData[5]<<8|pData[4];
  euler[2]=pData[7]<<8|pData[6];
  
  pitch = uint_to_float(euler[0], PITCH_CAN_MIN,PITCH_CAN_MAX, 16);
  yaw = uint_to_float(euler[1], YAW_CAN_MIN,YAW_CAN_MAX, 16);
  roll = uint_to_float(euler[2], ROLL_CAN_MIN,ROLL_CAN_MAX, 16);

  hasNewEuler = true;
}

void DM_Imu::CanRxCallback(uint32_t canId, uint8_t* data, uint8_t len)
{
  if (imuInstance && canId == imuInstance->canRxId)
  {
    imuInstance->ProcessFeedback(canId, data, len);
  }
}

void DM_Imu::GetEuler(float& _Pitch, float& _Roll, float& _Yaw)
{
  _Pitch = pitch;
  _Roll = roll;
  _Yaw = yaw;
  // 读完可以清标志（看你需求）
  hasNewEuler = false;
}



/**
************************************************************************
* @brief: float_to_uint: 浮点数转换为无符号整数函数
* @param[in]: x_float: 待转换的浮点数
* @param[in]: x_min: 范围最小值
* @param[in]: x_max: 范围最大值
* @param[in]: bits: 目标无符号整数的位数
* @retval: 无符号整数结果
* @details: 将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个
指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}


/**
************************************************************************
* @brief: uint_to_float: 无符号整数转换为浮点数函数
* @param[in]: x_int: 待转换的无符号整数
* @param[in]: x_min: 范围最小值
* @param[in]: x_max: 范围最大值
* @param[in]: bits: 无符号整数的位数
DM-IMU-L1 六轴 IMU 模块使用说明书 V1.1
第 17 页 共 22 页
* @retval: 浮点数结果
* @details: 将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结
果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /* converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


