/*===========================================================
* @file      B2MW_CANManager.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖:
* BspCan.h
* BspTimer.h
* MW_Common.hpp
* MW_RingBuffer.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* BSP层到中间件层的CAN总线外设管理模块
* 采用单例模式，统一管理和分配CAN的硬件资源,负责与BSP层沟通。
* 由于采取了单例模式，不应该会有多个实例，所以成员变量全部变成静态成员
* 1. 声明了 CanManager 类，作为CAN总线资源的统一管理者。
* 2. 定义了 USE_CanBus 枚举，用于表示上层中间件使用的CAN总线。
* 3. 定义了 MAX_CAN_SUBSCRIPTIONS 常量，用于表示CAN总线上最大可以挂的设备数量。
* 4. 定义了 CAN_TXQUEUE_SIZE 常量，用于表示CAN总线上最大可以发送的消息数量。
* ===========================================================
* @version   1.2
* @date      2025-11-14
* @copyright Copyright (c) 2025
============================================================*/
#ifndef B2MW_CANMANAGER_HPP
#define B2MW_CANMANAGER_HPP

/*========================= 文件依赖 =========================*/

#include "BspCan.h"
#include "BspTimer.h"
#include "MW_Common.hpp"
#include "MW_RingBuffer.hpp"

/*==================== CAN总线资源使用枚举 ====================*/

/**
 * @brief 上层中间件使用的CAN总线枚举
 */
enum  USE_CanBus : uint8_t
{   
    USE_CAN_BEGIN = 0,
    USE_CAN1 = 0,
    USE_CAN2 = 1,
    USE_CAN_END
};

/*========================== 宏定义 ==========================*/

/**
 * @brief CAN总线上最大可以挂的设备数量
 */
#define MAX_CAN_SUBSCRIPTIONS  12

/**
 * @brief CAN 管理类中发送队列的容量
 */
#define CAN_TXQUEUE_SIZE 12

/**
 * @brief CAN 标准ID的最大值 (11-bit)
 */
#define CAN_STANDARD_ID_MAX 0x7FF

/**
 * @brief CAN 发送邮箱的数目
 */
#define CAN_TXMAILBOX_NUM 3

/*======================= CAN 管理器类 =======================*/

/**
 * @brief CAN 管理器类
 * @details
 * 1. 采用单例模式，统一管理 CAN1 和 CAN2 资源。
 * 2. 负责初始化 BSP 层的 CAN，并设置硬件滤波器为“全部接收”模式。
 * 3. 提供基于静态数组的发布-订阅机制，实现零动态内存分配，保证实时性。
 * 4. 作为 BSP 和上层模块的桥梁，将收到的消息分发给对应的订阅者。
 */
class CanManager
{
public:

    /**
     * @brief 获取 CanManager 的单例实例
     */
    static CanManager& GetInstance();

    /**
     * @brief 申请CAN总线资源的接口,上层中间件申请CAN总线资源，申请成功，就可以调用StartResource方法启动资源
     * @param bus 要申请的总线
     * @param baudRate 要申请的波特率
     * @param mode 要申请的工作模式
     * @return 申请操作的状态
     * @details
     * 1. 检查总线是否已被申请,如果已被申请,则检查配置是否一致,如果不一致,则返回错误状态。
     * 2. 如果总线还没有被申请，则将申请总线的参数存储到CanManager的成员变量中
     */
    MW_Status AskResource(USE_CanBus bus, Can::CanBaudRate baudRate, Can::CanMode mode);    


    /**
     * @brief  在上层中间件向CanManager申请CAN总线资源后启动BSP层的CAN外设
     * @details
     * @return 初始化操作的状态
     * @details
     * 1. 如果总线还没有被初始化，就初始化CAN总线。
     * 2. 如果总线已经被初始化，则什么也不做。
     */
    MW_Status StartResource(USE_CanBus bus);
    

    /**
     * @brief 设备订阅指定CAN总线上的CAN消息并加入其回调函数到订阅列表中
     * @param bus 要订阅的总线
     * @param canId 要订阅的 CAN ID
     * @param callback 接收到消息时调用的回调函数
     * @return 订阅操作的状态
     */
    MW_Status Subscribe(USE_CanBus bus, uint32_t canId,CanRxCallback_t callback);

    
    /**
     * @brief 取消订阅(上层调用)
     * @param bus 要取消订阅的总线
     * @param canId 要取消订阅的 CAN ID
     * @param callback 要移除的回调函数 (用于区分同一ID的多个订阅者)
     * @return 取消订阅操作的状态
     */
    MW_Status UnSubscribe(USE_CanBus bus, uint32_t canId,  CanRxCallback_t callback);

    
    /**
     * @brief 向指定的CAN总线上的消息队列添加消息(上层调用)
     * @param bus 要使用的总线
     * @param msg 要发送的消息
     * @return 发送操作的状态
     */
    MW_Status sendMessage(USE_CanBus bus, const CanMessage& msg);

    
private:
    
/*==================== CAN 管理器私有成员变量 ====================*/
    /**
     * @brief CAN Manger管理BSP层的CAN1实例
     */
    Can Can1;
    
    /**
     * @brief CAN Manger管理BSP层的CAN2实例
     */
    Can Can2;    

    /**
     * @brief CAN Manger专用的基本定时器，用于处理CAN消息的发送
     */
    Timer Timer6;
    
    /**
     * @brief 上层中间件的订阅消息结构体
     */
    struct Subscription{
        uint32_t canId;
        CanRxCallback_t callback;
    };

    /**
     * @brief 用于发送CAN的消息队列数组
     */
    RingBuffer<CanMessage, CAN_TXQUEUE_SIZE> CanMsgSendQueue[USE_CAN_END];
    
    /**
     * @brief CAN_Resouce 存储CAN实例对象的指针数组
     * @details 记录CAN1和CAN2的实例指针,下标0为CAN1,下标1为CAN2
     */
    Can* CanResource[USE_CAN_END];

    /**
     * @brief 记录每个CAN是否需要启动
     * @details 下标0为CAN1,下标1为CAN2
     */
    bool NeedUSECAN[USE_CAN_END];
    
    /**
     * @brief 记录每个CAN是否已经初始化
     * @details 下标0为CAN1,下标1为CAN2
     */
    volatile bool CanIsInit[USE_CAN_END];

    /**
     * @brief 记录CAN Manger专用的基本定时器是否已经初始化
     */
    volatile bool TimIsInit;
    
    /**
     * @brief 记录每个CAN外设的工作模式配置
     * @details 下标0为CAN1,下标1为CAN2
     */
    Can::CanMode CanModeManager[USE_CAN_END];

    /**
     * @brief 记录每个CAN外设的波特率配置
     * @details 下标0为CAN1,下标1为CAN2
     */
    Can::CanBaudRate CanBaudRateManager[USE_CAN_END];
    
    /**
     * @brief 记录CAN1CallbackArray中的空闲起始位置
     */
    uint8_t Can1CallbackArrayIndex;
    
    /**
     * @brief 记录上层中间件在CAN1接受消息的回调函数,这两者是一一对应的关系
     */ 
    Subscription Can1CallbackArray[MAX_CAN_SUBSCRIPTIONS];

    /**
     * @brief 记录CAN2CallbackArray中的空闲起始位置
     */
    uint8_t Can2CallbackArrayIndex;
    
    /**
     * @brief 记录上层中间件在CAN2接受消息的回调函数,这两者是一一对应的关系
     */ 
    Subscription Can2CallbackArray[MAX_CAN_SUBSCRIPTIONS];


/*==================== CAN 管理器私有成员函数 ====================*/  
    /**
     * @brief 私有构造函数
     */
    CanManager();
    
    /**
     * @brief 析构默认
     */
    ~CanManager() = default;
    
    /**
     * @brief 拷贝构造私有
     */
    CanManager(const CanManager&);
    
    /**
     * @brief 赋值运算符构造私有
     */
    CanManager& operator=(const CanManager&);    

    /**
     * @brief CAN1 接收回调函数,所有中间件的回调函数实际上在这个上面调用
     * @param canId 收到的CAN ID
     * @param data 收到的CAN数据指针
     * @param len 收到的CAN数据长度
     */
    static void CAN1_RxCallback(uint32_t canId,  uint8_t* data, uint8_t len);
    
    /**
     * @brief CAN2 接收回调函数,所有中间件的回调函数实际上在这个上面调用
     * @param canId 收到的CAN ID
     * @param data 收到的CAN数据指针
     * @param len 收到的CAN数据长度
     */
    static void CAN2_RxCallback(uint32_t canId,  uint8_t* data, uint8_t len);

    
    /**
     * @brief 处理CAN发送队列，尝试从软件队列中发送消息
     * @details 1. 检查CAN消息队列中是否存在待发送的消息
     *          2. 如果存在，尝试从队列中取出消息并发送
     *          3. 如果CAN邮箱空闲，将消息发送到邮箱
     *          4. 如果CAN邮箱非空闲，就什么都不做
     */
    static void processCanSendQueue();

};

#endif /* B2MW_CANMANAGER_HPP */
