#include "Callback.h"
#include "BspUart.h"
#include "BspCan.h"
#include "BspSpi.h"

extern "C" 
{
#include "VOFA.h"
}



extern "C"
{
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == VOFA_UART) 
  {
    VOFA_RxCallBack();
  }
  else
  {
    Uart_RxEventCallback_Trampoline(huart, Size);
  }
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
  if (huart == VOFA_UART) 
  {
    TxCallBack_DoubleBufferUartDMA(&uartToVOFA);
  }
  else
  {
    Uart_TxCpltCallback_Trampoline(huart);
  }
}

// ==================== CAN 回调函数 ====================

/**
 * @brief CAN FIFO0 接收消息挂起回调
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  Can_RxFifo0Callback_Trampoline(hcan);
}

/**
 * @brief CAN FIFO1 接收消息挂起回调
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  Can_RxFifo1Callback_Trampoline(hcan);
}

/**
 * @brief CAN 发送邮箱0完成回调
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  Can_TxMailboxCallback_Trampoline(hcan, CAN_TX_MAILBOX0);
}

/**
 * @brief CAN 发送邮箱1完成回调
 */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  Can_TxMailboxCallback_Trampoline(hcan, CAN_TX_MAILBOX1);
}

/**
 * @brief CAN 发送邮箱2完成回调
 */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  Can_TxMailboxCallback_Trampoline(hcan, CAN_TX_MAILBOX2);
}

// ==================== SPI 回调函数 ====================

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  Spi_TxCpltCallback_Trampoline(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  Spi_RxCpltCallback_Trampoline(hspi);
}

} // extern "C"

