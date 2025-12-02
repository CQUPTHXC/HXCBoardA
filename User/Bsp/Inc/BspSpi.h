#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "common_inc.h"
#include "BspDevice.h"

#ifdef __cplusplus
extern "C" {
#endif

// SPI 传输完成回调函数类型
typedef void (*SpiTxRxCallback_t)(void);

void Spi_TxCpltCallback_Trampoline(void *_spiHandle);
void Spi_RxCpltCallback_Trampoline(void *_spiHandle);
void Spi_TxRxCpltCallback_Trampoline(void *_spiHandle);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief SPI 片选引脚配置结构体
 */
struct SpiCsPin
{
  GPIO_TypeDef* port;  // GPIO 端口 (GPIOA, GPIOB, GPIOC...)
  uint16_t pin;        // GPIO 引脚号 (0-15)
  
  /**
   * @brief 默认构造函数
   */
  SpiCsPin() : port(nullptr), pin(0) {}
  
  /**
   * @brief 带参数构造函数
   * @param _port GPIO 端口
   * @param _pin GPIO 引脚号
   */
  SpiCsPin(GPIO_TypeDef* _port, uint16_t _pin) : port(_port), pin(_pin) {}
};

class Spi
{
  // Trampoline 函数友元声明
  friend void Spi_TxCpltCallback_Trampoline(void *_spiHandle);
  friend void Spi_RxCpltCallback_Trampoline(void *_spiHandle);
  friend void Spi_TxRxCpltCallback_Trampoline(void *_spiHandle);

private:
  SPI_HandleTypeDef* hspi = nullptr;
  BspDevice_t deviceID;
  
  // 片选引脚配置
  SpiCsPin csPin;
  bool csPinEnabled = false;  // 是否启用片选引脚管理
  
  SpiTxRxCallback_t userTxCpltCallback = nullptr;
  SpiTxRxCallback_t userRxCpltCallback = nullptr;
  SpiTxRxCallback_t userTxRxCpltCallback = nullptr;

public:
  /**
   * @brief 构造函数
   * @param _deviceID SPI设备ID (DEVICE_SPI_1, DEVICE_SPI_2等)
   */
  explicit Spi(BspDevice_t _deviceID)
  {
    auto isValidDevice = (_deviceID >= DEVICE_SPI_START && _deviceID < DEVICE_SPI_END);
    if (isValidDevice)
    {
      deviceID = _deviceID;
    }
    else
    {
      deviceID = DEVICE_NONE;
      hspi = nullptr;
      return;
    }
  }

  // ==================== 核心功能 ====================
  
  /**
   * @brief 初始化SPI
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Init(GPIO_TypeDef* csPort, uint16_t csPin);

  /**
   * @brief 配置片选引脚
   * @param port GPIO 端口 (GPIOA, GPIOB, GPIOC...)
   * @param pin GPIO 引脚号 (0-15)
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> ConfigCsPin(GPIO_TypeDef* port, uint16_t pin);

  /**
   * @brief 拉低片选引脚（选中从设备）
   * @return BspResult<bool> 操作结果
   * @note 如果未配置片选引脚，返回错误
   */
  BspResult<bool> CsLow();

  /**
   * @brief 拉高片选引脚（释放从设备）
   * @return BspResult<bool> 操作结果
   * @note 如果未配置片选引脚，返回错误
   */
  BspResult<bool> CsHigh();

  /**
   * @brief 启动SPI
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Start();

  /**
   * @brief 停止SPI
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Stop();

  // ==================== 传输功能 ====================
  


  /**
   * @brief DMA方式发送数据
   * @param data 发送数据指针
   * @param size 数据长度
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> TransmitDMA(const uint8_t* data, uint16_t size);

  /**
   * @brief DMA方式接收数据
   * @param data 接收缓冲区指针
   * @param size 数据长度
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> ReceiveDMA(uint8_t* data, uint16_t size);

  /**
   * @brief DMA方式全双工传输
   * @param txData 发送数据指针
   * @param rxData 接收缓冲区指针
   * @param size 数据长度
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> TransmitReceiveDMA(const uint8_t* txData, uint8_t* rxData, uint16_t size);

 
  /**
   * @brief 设置发送完成回调
   * @param callback 回调函数
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SetTxCallback(SpiTxRxCallback_t callback);

  /**
   * @brief 设置接收完成回调
   * @param callback 回调函数
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SetRxCallback(SpiTxRxCallback_t callback);

  /**
   * @brief 设置收发完成回调
   * @param callback 回调函数
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SetTxRxCallback(SpiTxRxCallback_t callback);

  // ==================== 状态查询 ====================
  
  /**
   * @brief 检查SPI是否忙
   * @return BspResult<bool> 操作结果，成功返回true表示忙碌
   */
  BspResult<bool> IsBusy() const;

  /**
   * @brief 获取SPI状态
   * @return BspResult<uint32_t> 操作结果，成功返回HAL状态
   */
  BspResult<uint32_t> GetState() const;

  // ==================== 内部回调接口 ====================
  
  void InvokeTxCallback();
  void InvokeRxCallback();
  void InvokeTxRxCallback();
  BspResult<bool> ShowInfo() const;
};

#endif // __cplusplus

#endif // __BSP_SPI_H__