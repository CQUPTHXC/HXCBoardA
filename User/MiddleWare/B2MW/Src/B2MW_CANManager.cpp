/*===========================================================
* @file      B2MW_CANManager.cpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* MW_RingBuffer.hpp
* B2MW_CANManager.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* 1.实现了CANManager类的成员函数
* ===========================================================
* @version   1.2
* @date      2025-11-14
* @copyright Copyright (c) 2025
============================================================*/

/*========================= 文件依赖 ========================*/

#include "MW_RingBuffer.hpp"
#include "B2MW_CANManager.hpp"

/*================= CanManager的成员函数定义 =================*/

/** 
 * @brief 构造函数,初始化CanManager的成员变量 
 */
CanManager::CanManager():Can1(DEVICE_CAN_1),Can2(DEVICE_CAN_2),Timer6(DEVICE_TIMER_6),CanResource{&Can1,&Can2}
{
   /** 初始化CAN管理器的成员变量 */
   for(uint8_t i =USE_CAN_BEGIN ;i < USE_CAN_END; i++){
      NeedUSECAN[i] = false;
      CanIsInit[i] = false;
   }
   TimIsInit = false;
   /* 上层中间件调用的CAN回调函数数组索引清0 */
   Can1CallbackArrayIndex = 0;
   Can2CallbackArrayIndex = 0;
}; 

/**
 * @brief 获取CAN管理器的单例实例,返回当前CAN管理器的引用
 * @return CanManager& 当前CAN管理器的引用
 */
CanManager& CanManager::GetInstance()
{
    static CanManager instance;
    return instance;
}

/**
 * @brief 申请CAN总线资源的接口,上层中间件申请CAN总线资源，申请成功，就可以调用StartResource方法启动资源
 * @param bus 要申请的CAN总线
 * @param baudRate 要申请的波特率
 * @param mode 要申请的工作模式
 * @return MW_Status 申请结果
 *         返回值:
 *         INVALID_PARAM 表示参数无效,
 *         SUCCESS 表示申请成功
 */
MW_Status CanManager::AskResource(USE_CanBus bus, Can::CanBaudRate baudRate, Can::CanMode mode)
{ 
   /** 校验参数 */
   if(bus >= USE_CanBus::USE_CAN_END){
      return MW_Status::INVALID_PARAM;
   }

   /* 进入临界区 保证资源申请的原子性*/
   __disable_irq();
   
   /* 检查CAN总线是否已被申请,若被申请就检查是否与之前申请的配置一致 */
   if(NeedUSECAN[bus] == true){
      __enable_irq();
      /* 检查波特率是否一致 */
      if(CanBaudRateManager[bus] != baudRate){
         return MW_Status::INVALID_PARAM;
      }
      /* 检查工作模式是否一致 */
      if(CanModeManager[bus] != mode){
         return MW_Status::INVALID_PARAM;
      }
   }else{
      /* 若未被申请,则记录下申请的配置 */
      NeedUSECAN[bus] = true;
      CanBaudRateManager[bus] = baudRate;
      CanModeManager[bus] = mode;
      __enable_irq();
   }      
   
   return MW_Status::SUCCESS;
}


/**
 * @brief  在上层中间件向CanManager申请CAN总线资源后启动BSP层的CAN外设
 * @details 1. 依据上层中间件需要使用哪路CAN总线,就初始化哪路Can总线
 *          2. 如果使用CAN1，固定使用过滤器0，配置全通模式绑定FIFO 0
 *          3. 如果使用CAN2，固定使用过滤器14，配置全通模式绑定FIFO 0
 *          4. 初始化定时器以1kHZ频率触发中断
 *          5. 注册定时器中断回调函数processCanSendQueue      
 * @details
 *          1. 如果总线还没有被初始化，就初始化CAN总线。
 *          2. 如果总线已经被初始化，则什么也不做。 
 * @return 启动操作的状态
 *         返回值:
 *         INVALID_PARAM 表示参数无效,
 *         INVALID_OPERATION 表示Can没有成功,或者定时器初始化没有成功
 *         SUCCESS 表示启动成功
 */
MW_Status CanManager::StartResource(USE_CanBus bus){
   /*校验参数*/
   if(bus >=USE_CanBus::USE_CAN_END ){
      return MW_Status::INVALID_PARAM;
   }
   /*检查资源是否已被申请*/
   if (NeedUSECAN[bus] == false) {
      /* 如果未申请，则这是一个无效的操作顺序 */
      return MW_Status::INVALID_OPERATION;
   }
   BspResult<bool> OperationSuccess ;
   CanRxCallback_t CanRxCallback = (bus == USE_CAN1) ? CAN1_RxCallback : CAN2_RxCallback;
   
   /* 检查并更新CAN总线的初始化状态，此部分需要原子操作 */
   __disable_irq();
   bool should_init_can = (NeedUSECAN[bus] == true && CanIsInit[bus] == false);
   if (should_init_can) {
      CanIsInit[bus] = true; // 预先标记，防止其他线程重复初始化
   }
   __enable_irq();

   /* 需要初始化，但是还没有初始化*/
   if(should_init_can){
      OperationSuccess =CanResource[bus]->Init(CanBaudRateManager[bus],CanModeManager[bus]);
      /* 检查CAN总线初始化结果,如果失败返回无效参数 */
      if(!OperationSuccess.ok()){
            CanIsInit[bus] = false;
            /*能跑到这里说明用户在其他地方使用了Bsp层的Can资源,导致初始化失败*/
            return MW_Status::INVALID_OPERATION;
      }
      CanResource[bus]->SetRxFifo0Callback(CanRxCallback);
      CanResource[bus]->Start();
   }
   
   /* 检查并更新定时器的初始化状态，此部分需要原子操作 */
   __disable_irq();
   bool should_init_tim = (TimIsInit == false);
   if (should_init_tim) {
      TimIsInit = true; // 预先标记，防止其他线程重复初始化
   }
   __enable_irq();

   /* 如果定时器还没有初始化*/
   if(should_init_tim){
      /*初始化定时器以1000Hz频率触发中断*/
      OperationSuccess =Timer6.Init(1000);
      /* 检查定时器初始化结果,如果失败返回无效操作 */
      if(!OperationSuccess.ok()){
         TimIsInit = false;
         /*能跑到这里说明用户在其他地方使用了Bsp层的Timer资源,导致初始化失败*/
         return MW_Status::INVALID_OPERATION;
      }
      Timer6.SetCallback(processCanSendQueue);
      /*定时器启动*/
      Timer6.Start();
   }
   return MW_Status::SUCCESS;
};


/**
 * @brief 设备订阅指定CAN总线上的CAN消息，并加入其回调函数到订阅列表中
 * @param bus 要订阅的总线
 * @param canId 要订阅的CAN ID
 * @param callback 接收到消息时调用的回调函数 
 * @details 由于修改了回调数组这个静态变量，所以需要保证其原子性
 * @return 订阅操作的状态
 *         返回值:
 *         INVALID_PARAM 表示填入的参数无效,
 *         RESOURCE_BUSY 表示回调数组已满,
 *         SUCCESS 表示订阅成功
 */
MW_Status CanManager::Subscribe(USE_CanBus bus, uint32_t canId,CanRxCallback_t callback){
   /*校验参数*/
   if(bus >=USE_CanBus::USE_CAN_END ){
      return MW_Status::INVALID_PARAM;
   }
   if(CanIsInit[bus] == false){
      return MW_Status::INVALID_PARAM;
   }
   if(canId > CAN_STANDARD_ID_MAX){
      return MW_Status::INVALID_PARAM;
   }
   if(callback == nullptr){
      return MW_Status::INVALID_PARAM;
   }

   /*根据上层应用层提供的BUS选择操作资源*/
   Subscription* CallbackArray = (bus == USE_CAN1) ? Can1CallbackArray : Can2CallbackArray;
   uint8_t& CallbackArrayIndex = (bus == USE_CAN1) ? Can1CallbackArrayIndex : Can2CallbackArrayIndex;

   /*如果回调函数数组已满,则返回资源忙*/
   if(CallbackArrayIndex>=MAX_CAN_SUBSCRIPTIONS){
      return MW_Status::RESOURCE_BUSY;
   } 
   /*进入临界区*/
   __disable_irq();
   
   /*将canId和回调函数添加到回调数组中*/
   CallbackArray[CallbackArrayIndex].canId = canId;
   CallbackArray[CallbackArrayIndex].callback = callback;
   /*更新没有被使用的回调函数数组索引*/
   CallbackArrayIndex++;

   /*退出临界区*/
   __enable_irq();

   return MW_Status::SUCCESS;
};


/**
 * @brief 取消订阅(上层调用)
 * @param bus 要取消订阅的总线
 * @param canId 要取消订阅的 CAN ID
 * @param callback 要移除的回调函数 (用于区分同一ID的多个订阅者)
 * @details 由于修改了回调数组这个静态变量，所以需要保证其原子性
 * @return 取消订阅操作的状态
 *         返回值:
 *         INVALID_PARAM 表示参数无效,
 *         INVALID_OPERATION 表示没有找到匹配项,
 *         SUCCESS 表示取消订阅成功
*/
MW_Status CanManager::UnSubscribe(USE_CanBus bus, uint32_t canId, CanRxCallback_t callback){
   /*校验参数*/
   if(bus >=USE_CanBus::USE_CAN_END ){
      return MW_Status::INVALID_PARAM;
   }
   if(CanIsInit[bus] == false){
      return MW_Status::INVALID_PARAM;
   }
   if(canId > CAN_STANDARD_ID_MAX){
      return MW_Status::INVALID_PARAM;
   }
   if(callback == nullptr){
      return MW_Status::INVALID_PARAM;
   }

   /*标记是否找到匹配项*/
   bool found = false;

   /*根据上层应用层提供的BUS选择操作资源*/
   Subscription* CallbackArray = (bus == USE_CAN1) ? Can1CallbackArray : Can2CallbackArray;
   uint8_t& CallbackArrayIndex = (bus == USE_CAN1) ? Can1CallbackArrayIndex : Can2CallbackArrayIndex;
   
   /*进入临界区*/
   __disable_irq();
   
   /*遍历CallbackArray查找匹配的回调函数*/
   for(uint8_t i = 0; i<CallbackArrayIndex; i++){
      /*找到匹配项*/
      if(canId == CallbackArray[i].canId && callback == CallbackArray[i].callback){
         // 找到匹配项,将后续元素前移覆盖该元素
         CallbackArray[i] = CallbackArray[CallbackArrayIndex - 1];
         // 清空最后一个元素
         CallbackArray[CallbackArrayIndex-1].canId = 0;
         CallbackArray[CallbackArrayIndex-1].callback = nullptr;
         // 跟新没有被使用的回调函数数组索引
         CallbackArrayIndex--;
         found = true;
         break;
      }
   }
   
   /*退出临界区*/
   __enable_irq();
   /*找到了返回成功，否则返回无效操作*/
   return found ? MW_Status::SUCCESS : MW_Status::INVALID_OPERATION;
}


/**
 * @brief 发送CAN 消息到消息队列 (上层调用)
 * @param bus 要使用的总线
 * @param msg 要发送的消息
 * @return 发送操作的状态
 * @details 返回发送操作的状态,
 * INVALID_PARAM 表示参数无效,
 * RESOURCE_BUSY 表示消息队列已满,
 * SUCCESS 表示发送成功
 */
MW_Status CanManager::sendMessage(USE_CanBus bus, const CanMessage& msg){
   /*校验参数*/
   if(bus >=USE_CanBus::USE_CAN_END ){
      return MW_Status::INVALID_PARAM;
   }
   if(CanIsInit[bus] == false){
      return MW_Status::INVALID_PARAM;
   }
   if(msg.id > CAN_STANDARD_ID_MAX){
      return MW_Status::INVALID_PARAM;
   }
   
   MW_Status res = MW_Status::SUCCESS;
   /*根据上层应用层提供的BUS选择操作资源*/
   switch (bus)
   {
   case USE_CAN1:
      /*进入临界区, 确保发送操作的原子性*/
      __disable_irq();
      res = CanMsgSendQueue[USE_CAN1].push(msg);
      __enable_irq();
      break;
   case USE_CAN2:
      /*进入临界区, 确保发送操作的原子性*/
      __disable_irq();
      res = CanMsgSendQueue[USE_CAN2].push(msg);
      __enable_irq();
      break;
   default:
      res = MW_Status::INVALID_PARAM;
      break;
   }  
   return res;
}

/*==================== 私有函数实现 ====================*/



/**
 * @brief 注册在BSP层的CAN1接收回调函数，用于处理上层中间件订阅的CAN消息
 * @param canId 收到的CAN ID
 * @param data 收到的CAN数据指针
 * @param len 收到的CAN数据长度
 */
void CanManager::CAN1_RxCallback(uint32_t canId,  uint8_t* data, uint8_t len){
   /*获取单例*/
   CanManager& CanManagerInstance = CanManager::GetInstance(); 
   
   /*最多可能有 CAN_RXCALLBACK_ARRAYSIZE 个订阅者*/
    CanRxCallback_t callbacks_to_run[MAX_CAN_SUBSCRIPTIONS];
    uint8_t callbacks_count = 0;

    /*进入临界区, 快速查找并复制所有匹配的回调函数指针*/
    __disable_irq();
    for(uint8_t i = 0; i < CanManagerInstance.Can1CallbackArrayIndex; i++){
        if(canId == CanManagerInstance.Can1CallbackArray[i].canId){
                callbacks_to_run[callbacks_count++] = CanManagerInstance.Can1CallbackArray[i].callback;
        }
    }
    /*立即退出临界区, 恢复中断响应*/
    __enable_irq();

    /*在临界区外执行所有找到的回调*/
    for (uint8_t i = 0; i < callbacks_count; ++i) {
        if (callbacks_to_run[i] != nullptr) {
            callbacks_to_run[i](canId, data, len);
        }
    }
}


/**
 * @brief 注册在BSP层的CAN2接收回调函数，用于处理上层中间件订阅的CAN消息
 * @param canId 收到的CAN ID
 * @param data 收到的CAN数据指针
 * @param len 收到的CAN数据长度
 */
void CanManager::CAN2_RxCallback(uint32_t canId,  uint8_t* data, uint8_t len){
    /*获取单例*/
    CanManager& CanManagerInstance = CanManager::GetInstance();

    /*最多可能有 CAN_RXCALLBACK_ARRAYSIZE 个订阅者*/
    CanRxCallback_t callbacks_to_run[MAX_CAN_SUBSCRIPTIONS];
    uint8_t callbacks_count = 0;

    /*进入临界区, 快速查找并复制所有匹配的回调函数指针*/
    __disable_irq();
    /*遍历CAN2CallbackArray查找匹配的回调函数*/
    for(uint8_t i = 0; i < CanManagerInstance.Can2CallbackArrayIndex; i++){
        if(canId == CanManagerInstance.Can2CallbackArray[i].canId){
                callbacks_to_run[callbacks_count++] = CanManagerInstance.Can2CallbackArray[i].callback;
        }
    }
    /*立即退出临界区, 恢复中断响应*/
    __enable_irq();

    /*在临界区外执行所有找到的回调*/
    for (uint8_t i = 0; i < callbacks_count; ++i) {
        if (callbacks_to_run[i] != nullptr) {
            callbacks_to_run[i](canId, data, len);
        }
    }
}

/**
 * @brief TIM定时器中的回调函数，负责处理CAN消息队列中的待发送消息,发送周期1ms
 * @details 1. 检查CAN消息队列中是否存在待发送的消息
 *          2. 如果存在，尝试从队列中取出消息并发送
 *          3. 如果CAN邮箱空闲，将消息发送到邮箱
 *          4. 如果CAN邮箱非空闲，就什么都不做
 */
void CanManager::processCanSendQueue(){
   /*获取单例*/
   CanManager& CanManagerInstance = CanManager::GetInstance();
   /*临时变量*/
   CanMessage msg;
   /*用于存储CAN邮箱空闲数量的变量*/
   BspResult<uint32_t> FreeMailboxes;
   /*遍历所有Can总线，如果队列存在消息且CAN邮箱空闲，就从队列中取出一条消息发送*/
   for(uint8_t i = 0;i<USE_CAN_END;++i){

      if(CanManagerInstance.CanIsInit[i]==true){
         for(uint8_t j = 0;j<CAN_TXMAILBOX_NUM;++j){
            /*进入临界区, 确保消息发送的原子性*/
            __disable_irq();        
            /*获取空闲邮箱数目*/
            FreeMailboxes = CanManagerInstance.CanResource[i]->GetFreeTxMailboxes();
            /*如果存在空闲邮箱就从队列中取出一条消息发送*/
            if(FreeMailboxes.value>0 && !CanManagerInstance.CanMsgSendQueue[i].is_empty()){
               /*从队列中取出一条消息*/
               CanManagerInstance.CanMsgSendQueue[i].pop(msg); 
               /*退出临界区*/
               __enable_irq();
               /*尝试发送消息到CAN邮箱*/
               CanManagerInstance.CanResource[i]->SendMessage(msg);
            }else{
               /*如果CAN邮箱非空闲或者队列为空直接退出*/
               /*退出临界区*/
               __enable_irq();
               break;
            }   
         }
      }
   }
};

