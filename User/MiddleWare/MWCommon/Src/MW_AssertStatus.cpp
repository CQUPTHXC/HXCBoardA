/*===========================================================
* @file      MW_AssertStatus.cpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* MW_AssertStatus.hpp
* 该文件功能表述(先声明后定义)
* 1. 声明了断言失败后的回调函数
* 2. 定义了断言中间件状态失败后的处理函数(弱定义)
* ===========================================================
* @version   0.1
* @date      2025-10-14
* @copyright Copyright (c) 2025
============================================================*/

/*======================== 文件依赖 =========================*/
#include "MW_AssertStatus.hpp"

/*============== 断言中间件状态失败后的回调函数声明 ==============*/

void MW_AssertStatusFailedHandle(MW_Status status, const char* msg, const char* file, int line);


/*============== 断言中间件状态失败后的处理函数定义 ==============*/

/*
* @brief 断言中间件状态失败后的处理函数，负责调用回调函数
* @details
* 该函数在断言中间件状态失败时被调用
* 函数内容是 调用了断言失败回调函数
* @param status 断言中间件状态
* @param msg    断言失败的信息
* @param file   断言发生的文件名
* @param line   断言发生的行号
*/
void MW_AssertStatusFailed(MW_Status status, const char* msg, const char* file, int line){
    MW_AssertStatusFailedHandle(status, msg, file, line);
};


/*============= 断言中间件状态失败后的回调函数弱定义 =============*/

/**
 * @brief 断言中间件状态失败后的回调函数弱定义，在断言失败的地方会直接进入死循环
 * @details
 * 该函数在断言中间件状态失败时被调用(默认处理函数，如果用户没有定义的话)
 * 函数内容是 调用了断言失败回调函数
 * @param status 断言中间件状态
 * @param msg 断言失败的信息
 * @param file 断言发生的文件名
 * @param line 断言发生的行号
 */
__attribute__((weak)) void MW_AssertStatusFailedHandle(MW_Status status, const char* msg, const char* file, int line){
    while (1){};
}

/*============= 断言中间件状态失败后的回调函数强定义 =============*/


