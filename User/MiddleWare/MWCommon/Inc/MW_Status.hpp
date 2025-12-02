/*===========================================================
* @file      MW_Status.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖:
* cstdint
* ===========================================================
* 该文件功能表述(先声明后定义):
* 1.定义了中间件的通用状态
* 2.定义了中间件的函数执行状态
* 3.定义了中间件状态转换成字符串的函数
* ===========================================================
* @version   0.2
* @date      2025-10-14
* @copyright Copyright (c) 2025
============================================================*/
#ifndef MW_STATUS_HPP
#define MW_STATUS_HPP

/*======================== 文件依赖 =========================*/

#include <cstdint>

/*=================== 中间件的通用状态定义 =====================*/

/**
 * @brief 中间件通用状态模块
 * @details
 *  MW_Status::SUCCESS-操作成功
 *  MW_Status::ERROR-通用错误
 *  MW_Status::TIMEOUT-操作超时
 *  MW_Status::INVALID_PARAM-无效参数
 *  MW_Status::UNINITIALIZED-未初始化
 *  MW_Status::RESOURCE_BUSY-资源忙
 */
enum class MW_Status : uint8_t
{
    SUCCESS         = 0x00,  /*!< 操作成功 */
    ERROR           = 0x01,  /*!< 通用错误 */
    TIMEOUT         = 0x02,  /*!< 操作超时 */
    INVALID_PARAM   = 0x03,  /*!< 无效参数 */
    UNINITIALIZED   = 0x04,  /*!< 未初始化 */
    RESOURCE_BUSY   = 0x05,  /*!< 资源已占用 */
    INVALID_OPERATION = 0x06, /*!< 无效操作 */
};

/*=================== 中间件通用函数返回类 ======================*/

/**
 * @brief 中间件函数执行状态结构体
 * @tparam T 函数返回值类型
 * @tparam status 中间件状态
 */
template<class T>
struct MW_FuncStatus{
    T result;
    MW_Status status;
};

/*===================== 中间件状态转换函数 =====================*/

/**
 * @brief 将 MW_Status 转换为字符串
 * @param status 要转换的状态
 * @return const char* 状态的字符串表示
 */
inline const char* MW_StatusToString(MW_Status status)
{
    switch (status)
    {
        case MW_Status::SUCCESS:           return "SUCCESS";
        case MW_Status::ERROR:             return "ERROR";
        case MW_Status::TIMEOUT:           return "TIMEOUT";
        case MW_Status::INVALID_PARAM:     return "INVALID_PARAM";
        case MW_Status::UNINITIALIZED:     return "UNINITIALIZED";
        case MW_Status::RESOURCE_BUSY:     return "RESOURCE_BUSY";
        case MW_Status::INVALID_OPERATION: return "INVALID_OPERATION";
        default:                           return "UNKNOWN";
    }
}

#endif /* MW_STATUS_HPP */
