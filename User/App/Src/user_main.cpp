
#include <limits>

#include "UserMain.h" // 包含用户主函数头文件
#include "BspDevice.h"
#include "BspTimer.h"
#include "BspUart.h"
#include "BspPwm.h"
#include "BspCan.h"
#include "Log.h"
extern "C" 
{
  #include "VOFA.h" 
  #include "task.h"
}

Uart uartDebug(DEVICE_USART_4);

void userMain() 
{
  float testNumber;
  uint32_t counter = 0;

  Log::Init(uartDebug);
  Log::RegisterData_Vofa("testNumber", &testNumber);
  
  while (1)
  {
    counter++;
    Log::Print("%d\n", counter);
    Delay(1);
  }
}

