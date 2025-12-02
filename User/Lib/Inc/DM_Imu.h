#ifndef __DM_IMU_H__
#define __DM_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#ifdef __cplusplus


#include "common_inc.h"
#include "B2MW_CANManager.hpp"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN (-58.8f)
#define GYRO_CAN_MAX (34.88f)
#define GYRO_CAN_MIN (-34.88f)
#define PITCH_CAN_MAX (90.0f)
#define PITCH_CAN_MIN (-90.0f)
#define ROLL_CAN_MAX (180.0f)
#define ROLL_CAN_MIN (-180.0f)
#define YAW_CAN_MAX (180.0f)
#define YAW_CAN_MIN (-180.0f)
#define TEMP_MIN (0.0f)
#define TEMP_MAX (60.0f)
#define Quaternion_MIN (-1.0f)
#define Quaternion_MAX (1.0f)

class DM_Imu
{
private:

  USE_CanBus bus = USE_CAN1;

  bool hasNewEuler = false;

  float pitch = 0.0f;
  float roll  = 0.0f;
  float yaw   = 0.0f;

public:
  explicit DM_Imu(USE_CanBus _bus = USE_CAN1);
  ~DM_Imu();

  uint32_t canRxId = 0x011;

  bool Init();

  bool RequestEuler(uint8_t reg = 0x03); // 默认请求欧拉角数据寄存器0x03
  bool HasNewEuler() const { return hasNewEuler; }
  
  void GetEuler(float& _Pitch, float& _Roll, float& _Yaw);
  
  void ProcessFeedback(uint32_t canId, uint8_t* data, uint8_t len);
  static void CanRxCallback(uint32_t canId, uint8_t* data, uint8_t len);
};


int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);


#endif // __cplusplus

#endif // !__DM_IMU_H__
