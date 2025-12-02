/*===========================================================
* @file      MW_AssertStatus.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* MW_Status.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* 1.声明了断言中间件状态失败后的处理函数
* 2.定义了断言中间件状态的开关
* 3.定义了断言中间件状态的宏函数
* ===========================================================
* @version   0.3
* @date      2025-10-27
* @copyright Copyright (c) 2025
============================================================*/
#ifndef MW_ASSERTSTATUS_HPP
#define MW_ASSERTSTATUS_HPP

/*======================== 文件依赖 =========================*/
#include "MW_Status.hpp"

/*=============== 断言中间件状态失败后的函数声明 ================*/

/**
 * @brief 声明了断言中间件状态失败后的处理函数
 * @details
 * 该函数在断言中间件状态失败时被调用
 * 它用于处理断言失败的情况,例如打印错误信息、触发异常等
 * @param status 断言中间件状态
 * @param msg    断言失败的信息,格式为"[模块名] 具体错误信息"
 * @param file   断言发生的文件名
 * @param line   断言发生的行号
 */
void MW_AssertStatusFailed(MW_Status status, const char* msg, const char* file, int line);

/*=================== 断言中间件状态开关定义 ===================*/

/**
 * @brief 断言中间件状态开关
 * @details
 *  MW_ASSERTSTATUS_ENABLE-1:开启断言中间件状态检查
 *  MW_ASSERTSTATUS_ENABLE-0:关闭断言中间件状态检查
 */
#define MW_ASSERTSTATUS_ENABLE 1  


/*=================== 断言中间件状态函数定义 ===================*/

/**
 * @brief 通用模块断言宏 - 需要在使用前定义模块名
 * @details
 * 该宏用于在模块中检查中间件状态是否成功
 * 如果状态失败,则调用断言失败处理函数
 * @param MODULE_NAME 模块名,用于在断言失败信息中标识模块
 * @param status 断言中间件状态
 * @param msg    断言失败的信息,格式为"具体错误信息"
 * @details
 * 1. 如果断言中间件状态(status)不是MW_Status::SUCCESS,则调用MW_AssertStatusFailed函数
 * 2. 该函数会打印出模块名、断言失败信息、文件名和行号
 */
#if MW_ASSERTSTATUS_ENABLE
#define MODULE_ASSERT(MODULE_NAME, status, msg) \
    do { \
        if ((status) != MW_Status::SUCCESS) { \
            MW_AssertStatusFailed((status), "[" #MODULE_NAME "] " msg, __FILE__, __LINE__); \
        } \
    } while(0)
#else
    #define MODULE_ASSERT(MODULE_NAME, status, msg) ((void)0)
#endif

/*=================== 使用说明 ===================*/
/*
 * 在各模块头文件中定义自己的断言宏，例如：
 * 
 * // 在 DJI3508.hpp 中：
 * #define DJI3508_ASSERT_ENABLE 1
 * #if (DJI3508_ASSERT_ENABLE == 1)
 *     #define DJI3508_ASSERT(status, msg) MODULE_ASSERT(DJI3508, status, msg)
 * #else
 *     #define DJI3508_ASSERT(status, msg) ((void)0)
 * #endif
 * 
 * // 在 CAN_Manager.hpp 中：
 * #define CAN_ASSERT_ENABLE 1
 * #if (CAN_ASSERT_ENABLE == 1)
 *     #define CAN_ASSERT(status, msg) MODULE_ASSERT(CAN, status, msg)
 * #else
 *     #define CAN_ASSERT(status, msg) ((void)0)
 * #endif
 */


#endif /* MW_ASSERTSTATUS_HPP */