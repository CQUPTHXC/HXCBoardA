#include "BspSpi.h"
#include <cstdint>
#include <cstdio>

extern "C"
{
#include "VOFA.h"
}

static Spi* spiInstances[DEVICE_SPI_END - DEVICE_SPI_START] = {nullptr}; // 全局单例指针

static const char* SpiInstanceName(const SPI_TypeDef* instance)
{
  if (instance == SPI1) return "SPI1";
  if (instance == SPI2) return "SPI2";
  if (instance == SPI3) return "SPI3";
#ifdef SPI4
  if (instance == SPI4) return "SPI4";
#endif
#ifdef SPI5
  if (instance == SPI5) return "SPI5";
#endif
#ifdef SPI6
  if (instance == SPI6) return "SPI6";
#endif
  return "UNKNOWN";
}

static const char* SpiModeToString(uint32_t mode)
{
  switch (mode)
  {
  case SPI_MODE_MASTER: return "MASTER";
  case SPI_MODE_SLAVE:  return "SLAVE";
  default:              return "UNKNOWN";
  }
}

static const char* SpiNssToString(uint32_t nss)
{
  switch (nss)
  {
  case SPI_NSS_SOFT:           return "SOFT";
  case SPI_NSS_HARD_INPUT:     return "HARD_INPUT";
  case SPI_NSS_HARD_OUTPUT:    return "HARD_OUTPUT";
#ifdef SPI_NSS_SOFT_POS
  case SPI_NSS_SOFT_POS:       return "SOFT_POS";
#endif
  default:                     return "UNKNOWN";
  }
}

static const char* SpiDirectionToString(uint32_t direction)
{
  switch (direction)
  {
  case SPI_DIRECTION_2LINES:        return "2LINES";
  case SPI_DIRECTION_2LINES_RXONLY: return "2LINES_RXONLY";
  case SPI_DIRECTION_1LINE:         return "1LINE";
  default:                          return "UNKNOWN";
  }
}

static const char* SpiDataSizeToString(uint32_t dataSize)
{
#ifdef SPI_DATASIZE_4BIT
  if (dataSize == SPI_DATASIZE_4BIT)  return "4BIT";
#endif
#ifdef SPI_DATASIZE_5BIT
  if (dataSize == SPI_DATASIZE_5BIT)  return "5BIT";
#endif
#ifdef SPI_DATASIZE_6BIT
  if (dataSize == SPI_DATASIZE_6BIT)  return "6BIT";
#endif
#ifdef SPI_DATASIZE_7BIT
  if (dataSize == SPI_DATASIZE_7BIT)  return "7BIT";
#endif
  if (dataSize == SPI_DATASIZE_8BIT)  return "8BIT";
#ifdef SPI_DATASIZE_9BIT
  if (dataSize == SPI_DATASIZE_9BIT)  return "9BIT";
#endif
#ifdef SPI_DATASIZE_10BIT
  if (dataSize == SPI_DATASIZE_10BIT) return "10BIT";
#endif
#ifdef SPI_DATASIZE_11BIT
  if (dataSize == SPI_DATASIZE_11BIT) return "11BIT";
#endif
#ifdef SPI_DATASIZE_12BIT
  if (dataSize == SPI_DATASIZE_12BIT) return "12BIT";
#endif
#ifdef SPI_DATASIZE_13BIT
  if (dataSize == SPI_DATASIZE_13BIT) return "13BIT";
#endif
#ifdef SPI_DATASIZE_14BIT
  if (dataSize == SPI_DATASIZE_14BIT) return "14BIT";
#endif
#ifdef SPI_DATASIZE_15BIT
  if (dataSize == SPI_DATASIZE_15BIT) return "15BIT";
#endif
  if (dataSize == SPI_DATASIZE_16BIT) return "16BIT";
  return "UNKNOWN";
}

static const char* SpiStateToString(HAL_SPI_StateTypeDef state)
{
  switch (state)
  {
  case HAL_SPI_STATE_RESET:      return "RESET";
  case HAL_SPI_STATE_READY:      return "READY";
  case HAL_SPI_STATE_BUSY:       return "BUSY";
  case HAL_SPI_STATE_BUSY_TX:    return "BUSY_TX";
  case HAL_SPI_STATE_BUSY_RX:    return "BUSY_RX";
  case HAL_SPI_STATE_BUSY_TX_RX: return "BUSY_TX_RX";
  case HAL_SPI_STATE_ERROR:      return "ERROR";
  default:                       return "UNKNOWN";
  }
}

static const char* GpioPortName(const GPIO_TypeDef* port)
{
#ifdef GPIOA
  if (port == GPIOA) return "GPIOA";
#endif
#ifdef GPIOB
  if (port == GPIOB) return "GPIOB";
#endif
#ifdef GPIOC
  if (port == GPIOC) return "GPIOC";
#endif
#ifdef GPIOD
  if (port == GPIOD) return "GPIOD";
#endif
#ifdef GPIOE
  if (port == GPIOE) return "GPIOE";
#endif
#ifdef GPIOF
  if (port == GPIOF) return "GPIOF";
#endif
#ifdef GPIOG
  if (port == GPIOG) return "GPIOG";
#endif
#ifdef GPIOH
  if (port == GPIOH) return "GPIOH";
#endif
#ifdef GPIOI
  if (port == GPIOI) return "GPIOI";
#endif
  return "UNKNOWN";
}

static uint32_t GpioPinIndex(uint16_t pinMask)
{
  for (uint32_t idx = 0; idx < 16; ++idx)
  {
    if (pinMask & (1U << idx))
    {
      return idx;
    }
  }
  return 0xFFFFFFFFU;
}

BspResult<bool> Spi::Init(GPIO_TypeDef* csPort, uint16_t csPin)
{
  BSP_CHECK(deviceID >= DEVICE_SPI_START && deviceID < DEVICE_SPI_END, BspError::InvalidDevice, bool);
  BSP_CHECK(csPort != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(csPin <= GPIO_PIN_15, BspError::InvalidParam, bool);

  auto handleResult = Bsp_GetDeviceHandle(deviceID);
  BSP_CHECK(handleResult.ok() && handleResult.value != nullptr, handleResult.error, bool);
  hspi = static_cast<SPI_HandleTypeDef*>(handleResult.value);
  BSP_CHECK(hspi->Instance != nullptr, BspError::InvalidDevice, bool);

  HAL_StatusTypeDef status = HAL_SPI_Init(hspi);
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);

  auto result = Bsp_StartDevice(deviceID);
  BSP_CHECK(result.ok(), result.error, bool);

  auto configResult = ConfigCsPin(csPort, csPin);
  BSP_CHECK(configResult.ok(), configResult.error, bool);

  userRxCpltCallback = nullptr;
  userTxCpltCallback = nullptr;
  userTxRxCpltCallback = nullptr;

  spiInstances[deviceID - DEVICE_SPI_START] = this; // 注册实例指针

  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::ConfigCsPin(GPIO_TypeDef* port, uint16_t pin)
{
  BSP_CHECK(port != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(pin <= GPIO_PIN_15, BspError::InvalidParam, bool);

  csPin = {port, pin};

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &GPIO_InitStruct);

  // 默认拉高 CS(片选无�?
  csPinEnabled = true;

  CsHigh();


  return BspResult<bool>::success(true);
}


BspResult<bool> Spi::CsLow()
{
  BSP_CHECK(csPinEnabled, BspError::InvalidParam, bool);
  csPin.port->BSRR = 1U << (csPin.pin + 16); // 拉低
  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::CsHigh()
{
  BSP_CHECK(csPinEnabled, BspError::InvalidParam, bool);
  csPin.port->BSRR = 1U << csPin.pin; // 拉高
  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::Stop()
{
  BSP_CHECK(hspi != nullptr, BspError::NullHandle, bool);
  
  if (HAL_SPI_DeInit(hspi) != HAL_OK)
  {
    BSP_RETURN_FAILURE(BspError::HalError, bool);
  }
  
  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::TransmitDMA(const uint8_t* data, uint16_t size)
{
  BSP_CHECK(hspi != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(data != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(size > 0, BspError::InvalidParam, bool);

  if (HAL_SPI_Transmit_DMA(hspi, const_cast<uint8_t*>(data), size) != HAL_OK)
  {
    BSP_RETURN_FAILURE(BspError::HalError, bool);
  }

  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::ReceiveDMA(uint8_t* data, uint16_t size)
{
  BSP_CHECK(hspi != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(data != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(size > 0, BspError::InvalidParam, bool);

  if (HAL_SPI_Receive_DMA(hspi, data, size) != HAL_OK)
  {
    BSP_RETURN_FAILURE(BspError::HalError, bool);
  }

  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::TransmitReceiveDMA(const uint8_t* txData, uint8_t* rxData, uint16_t size)
{
  BSP_CHECK(hspi != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(txData != nullptr && rxData != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(size > 0, BspError::InvalidParam, bool);

  if (HAL_SPI_TransmitReceive_DMA(hspi, const_cast<uint8_t*>(txData), rxData, size) != HAL_OK)
  {
    BSP_RETURN_FAILURE(BspError::HalError, bool);
  }

  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::SetTxCallback(SpiTxRxCallback_t callback)
{
  userTxCpltCallback = callback;
  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::SetRxCallback(SpiTxRxCallback_t callback)
{
  userRxCpltCallback = callback;
  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::SetTxRxCallback(SpiTxRxCallback_t callback)
{
  userTxRxCpltCallback = callback;
  return BspResult<bool>::success(true);
}

BspResult<bool> Spi::IsBusy() const
{
  BSP_CHECK(hspi != nullptr, BspError::NullHandle, bool);
  
  HAL_SPI_StateTypeDef state = HAL_SPI_GetState(hspi);
  bool isBusy = (state != HAL_SPI_STATE_READY);
  
  return BspResult<bool>::success(isBusy);
}

BspResult<uint32_t> Spi::GetState() const
{
  BSP_CHECK(hspi != nullptr, BspError::NullHandle, uint32_t);
  
  uint32_t state = static_cast<uint32_t>(HAL_SPI_GetState(hspi));
  return BspResult<uint32_t>::success(state);
}

BspResult<bool> Spi::ShowInfo() const
{
  BSP_CHECK(hspi != nullptr, BspError::NullHandle, bool);

  SPI_HandleTypeDef* handle = hspi;
  char csInfo[32];
  char csLevel[8];
  std::snprintf(csInfo, sizeof(csInfo), "NONE");
  std::snprintf(csLevel, sizeof(csLevel), "-");

  if (csPinEnabled && csPin.port != nullptr)
  {
    uint32_t pinIndex = GpioPinIndex(csPin.pin);
    if (pinIndex <= 15U)
    {
      std::snprintf(csInfo, sizeof(csInfo), "%s P%d", GpioPortName(csPin.port), pinIndex);
    }
    else
    {
      std::snprintf(csInfo, sizeof(csInfo), "%s 0x%04X", GpioPortName(csPin.port), csPin.pin);
    }
    std::snprintf(csLevel, sizeof(csLevel), "%s", HAL_GPIO_ReadPin(csPin.port, csPin.pin) == GPIO_PIN_RESET ? "LOW" : "HIGH");
  }

  ::Printf("===== %s Info =====\n"
        "Device ID: %d\n"
        "Mode: %s\n"
        "Direction: %s\n"
        "DataSize: %s\n"
        "BaudPrescaler: %u\n"
        "FirstBit: %s\n"
        "NSS: %s\n"
        "State: %s\n"
        "ErrorCode: 0x%08lX\n"
        "TIMode: %s\n"
        "CRCPolynomial: %u\n"
        "CS Pin: %s\n"
        "CS Level: %s\n"
        "DMA[T:R]: %s/%s\n"
        "Callbacks[T:R:TR]: %s/%s/%s\n"
        "=======================\n",
        SpiInstanceName(handle->Instance),
        deviceID,
        SpiModeToString(handle->Init.Mode),
        SpiDirectionToString(handle->Init.Direction),
        SpiDataSizeToString(handle->Init.DataSize),
        static_cast<unsigned>(handle->Init.BaudRatePrescaler),
        handle->Init.FirstBit == SPI_FIRSTBIT_MSB ? "MSB" : "LSB",
        SpiNssToString(handle->Init.NSS),
        SpiStateToString(HAL_SPI_GetState(handle)),
        static_cast<unsigned long>(HAL_SPI_GetError(handle)),
        handle->Init.TIMode == SPI_TIMODE_DISABLE ? "DIS" : "EN",
        handle->Init.CRCPolynomial,
        csInfo,
        csLevel,
        handle->hdmatx ? "Y" : "N",
        handle->hdmarx ? "Y" : "N",
        userTxCpltCallback ? "Y" : "N",
        userRxCpltCallback ? "Y" : "N",
        userTxRxCpltCallback ? "Y" : "N"
  );
  Delay(500);
  return BspResult<bool>::success(true);
}

void Spi::InvokeTxCallback()
{
  if (userTxCpltCallback != nullptr)
  {
    userTxCpltCallback();
  }
}

void Spi::InvokeRxCallback()
{
  if (userRxCpltCallback != nullptr)
  {
    userRxCpltCallback();
  }
}

void Spi::InvokeTxRxCallback()
{
  if (userTxRxCpltCallback != nullptr)
  {
    userTxRxCpltCallback();
  }
}

// Trampoline 函数实现
extern "C"
{
  void Spi_TxCpltCallback_Trampoline(void *_spiHandle)
  {
    SPI_HandleTypeDef* hspi = static_cast<SPI_HandleTypeDef*>(_spiHandle);
    for (uint8_t i = 0; i < DEVICE_SPI_END - DEVICE_SPI_START; i++)
    {
      if (spiInstances[i] != nullptr && spiInstances[i]->hspi == hspi)
      {
        spiInstances[i]->InvokeTxCallback();
        break;
      }
    }
  }

  void Spi_RxCpltCallback_Trampoline(void *_spiHandle)
  {
    SPI_HandleTypeDef* hspi = static_cast<SPI_HandleTypeDef*>(_spiHandle);
    for (uint8_t i = 0; i < DEVICE_SPI_END - DEVICE_SPI_START; i++)
    {
      if (spiInstances[i] != nullptr && spiInstances[i]->hspi == hspi)
      {
        spiInstances[i]->InvokeRxCallback();
        break;
      }
    }
  }

  void Spi_TxRxCpltCallback_Trampoline(void *_spiHandle)
  {
    SPI_HandleTypeDef* hspi = static_cast<SPI_HandleTypeDef*>(_spiHandle);
    for (uint8_t i = 0; i < DEVICE_SPI_END - DEVICE_SPI_START; i++)
    {
      if (spiInstances[i] != nullptr && spiInstances[i]->hspi == hspi)
      {
        spiInstances[i]->InvokeTxRxCallback();
        break;
      }
    }
  }
}
