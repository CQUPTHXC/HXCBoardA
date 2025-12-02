#include "common_inc.h"
#include "ws2812.h"
#include "tim.h" // 包含定时器头文件
#include "BspPwm.h"

// WS2812 时序参数
// WS2812需要800kHz的PWM频率 (1.25μs周期)
// 0码: 高电平0.4μs, 低电平0.85μs (占空比约32%)
// 1码: 高电平0.8μs, 低电平0.45μs (占空比约64%)
#define WS2812_PWM_FREQ 800000  // 800kHz

// PWM数据缓冲区
static uint16_t pwm_buf[WS2812_DATA_LEN] = {0};

// PWM对象
static Pwm pwm(DEVICE_PWM_12); // 使用PWM设备12 (TIM12)

// 用于存储CODE_0和CODE_1的值 (根据实际ARR动态计算)
static uint32_t CODE_0 = 0;
static uint32_t CODE_1 = 0;

/**
 * @brief WS2812初始化函数
 * @note 初始化PWM,设置频率和占空比参数
 */
void ws2812_init(void)
{
    // 初始化PWM,设置800kHz频率
    auto initResult = pwm.Init(WS2812_PWM_FREQ);
    if (!initResult.ok())
    {
        // 初始化失败处理
        return;
    }

    // 获取ARR值以计算占空比
    auto arrResult = pwm.GetARR();
    if (!arrResult.ok())
    {
        return;
    }
    uint32_t arr = arrResult.value;

    // 计算0码和1码对应的CCR值
    // 0码占空比约32% (高电平0.4μs)
    CODE_0 = (uint32_t)(arr * 0.32f);
    // 1码占空比约64% (高电平0.8μs)
    CODE_1 = (uint32_t)(arr * 0.64f);

    // 初始化PWM缓冲区为全0 (复位信号)
    for (int i = 0; i < WS2812_DATA_LEN; i++)
    {
        pwm_buf[i] = 0;
    }
    
    // 启动PWM通道2
    pwm.Start(Pwm::CHANNEL_2);
}

/**
 * @brief 设置单个LED的颜色
 * @param led_id LED编号 (0~LED_NUM-1)
 * @param r 红色值 (0~255)
 * @param g 绿色值 (0~255)
 * @param b 蓝色值 (0~255)
 * @note WS2812的颜色顺序是GRB
 */
void ws2812_set_color(uint16_t led_id, uint8_t r, uint8_t g, uint8_t b)
{
    if (led_id >= LED_NUM)
    {
        return; // 超出范围
    }

    // WS2812的颜色顺序是GRB
    uint32_t color = (g << 16) | (r << 8) | b;
    uint16_t *p = &pwm_buf[led_id * 24];

    // 将24位颜色数据转换为PWM占空比
    for (int i = 23; i >= 0; i--)
    {
        if ((color >> i) & 1)
        {
            *p = CODE_1; // 1码
        }
        else
        {
            *p = CODE_0; // 0码
        }
        p++;
    }
}

/**
 * @brief 将颜色数据发送到LED灯带 (不使用DMA，软件方式)
 * @note 通过逐位设置PWM占空比并延时来发送数据
 */
void ws2812_show(void)
{
    // 禁用中断，确保时序准确
    __disable_irq();
    
    // 发送所有PWM数据
    for (uint16_t i = 0; i < WS2812_DATA_LEN; i++)
    {
        // 设置当前位的占空比
        pwm.SetDutyTicks(Pwm::CHANNEL_2, pwm_buf[i]);
        
        // 等待1.25μs (800kHz周期)
        // 假设系统时钟168MHz，需要约210个时钟周期
        for (volatile uint32_t j = 0; j < 35; j++);
    }
    
    // 发送完成后设置为0，确保复位信号
    pwm.SetDutyTicks(Pwm::CHANNEL_2, 0);
    
    // 恢复中断
    __enable_irq();
}


