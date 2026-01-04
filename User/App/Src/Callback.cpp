#include "Callback.h"
#include "BspUart.h"
#include "BspCan.h"
#include "BspSpi.h"
#include "BspGpio.h"

extern "C" 
{
}



extern "C"
{
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

  Uart_RxEventCallback_Trampoline(huart, Size);

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Uart_ErrorCallback_Trampoline(huart);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
  Uart_TxCpltCallback_Trampoline(huart);
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

//  =================== GPIO 回调函数 ==================

/**
 * @brief HAL GPIO EXTI回调函数（重写HAL库的弱定义）
 * @param GPIO_Pin 触发中断的GPIO引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Gpio_ExtiCallback_Trampoline(GPIO_Pin);
}

} // extern "C"

