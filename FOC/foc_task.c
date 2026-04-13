#include "foc_task.h"

#include <math.h>
#include "esp_attr.h"
#include "BSPDriver/MT6701.h"
#include "foc_calibrate.h"
#include "foc_lib.h"

static uint16_t s_zero_electric_angle_u16 = 0U;

static void foc_task_sync_calibration_cache(void)
{
    s_zero_electric_angle_u16 = foc_float_to_angle_u16(foc_params.zero_electric_angle);
}

void foc_task_init(void)
{
    foc_task_sync_calibration_cache();
}

void IRAM_ATTR update_current_sensor(float electrical_angle)
{
    (void)electrical_angle;

    // 电流传感器尚未接入，当前只保留字段更新位置，不进入电流闭环。
    // 后面接入 ADC 采样后，可以直接在这里填充 iu/iv/iw、i_alpha/i_beta、id/iq。
    foc_params.iu = 0.0f;
    foc_params.iv = 0.0f;
    foc_params.iw = 0.0f;
    foc_params.i_alpha = foc_params.iu;
    foc_params.i_beta = (foc_params.iu + (2.0f * foc_params.iv)) * ONE_BY_SQRT3;
    foc_params.id = 0.0f;
    foc_params.iq = 0.0f;
}

float IRAM_ATTR foc_placeholder_get_shaft_angle(void)
{
    // 真实机械角读取来自 MT6701，原始 14bit 角度扩展到 16bit 环形角度。
    const uint16_t raw = MT6701_GetRawData();
    foc_params.shaft_angle_u16 = (uint16_t)(raw << 2);
    return foc_angle_u16_to_float(foc_params.shaft_angle_u16);
}

float IRAM_ATTR foc_electrical_angle_from_shaft(float shaft_angle)
{
    // 电角度 = 机械角 * 极对数 * 方向 - 零电角
    // 这里全部在 16bit 角度环里做，溢出自动等价于 2pi 回绕。
    const int32_t shaft_u16 = (int32_t)foc_params.shaft_angle_u16;
    const int32_t direction = (foc_params.sensor_direction >= 0) ? 1 : -1;
    const int32_t electrical_u16 = (shaft_u16 * foc_params.pole_pairs * direction) - s_zero_electric_angle_u16;

    (void)shaft_angle;
    foc_params.electrical_angle_u16 = (uint16_t)electrical_u16;
    return foc_angle_u16_to_float(foc_params.electrical_angle_u16);
}

void IRAM_ATTR foc_fast_loop_step_isr(void)
{
    // 检查 FOC 状态机：如果未启用、未启动（使能）或者未校准，则直接退出不输出电压
    if (!foc_enabled || !foc_started || !foc_calibrated) {
        return;
    }

    // --- FOC 核心快环处理步骤 ---
    
    // 1. 获取转子当前物理机械角 
    // 调用 MT6701 磁编码器接口，获取并计算出当前轴的机械角度 (范围 0~2pi)
    foc_params.shaft_angle = foc_placeholder_get_shaft_angle();
    
    // 2. 由机械角推导电角度
    // 电角度 = (机械角 * 极对数 * 传感器方向) - 零电角度
    // 这一步用于将转子的物理空间位置对应到电磁场的电周期内，以便定子磁场能完美滞后/超前转子磁场
    foc_params.electrical_angle = foc_electrical_angle_from_shaft(foc_params.shaft_angle);
    
    // 3. 更新当前相电流采样数据的占位（此时处于开环，实际如果接入ADC闭环则在这里更新）
    update_current_sensor(foc_params.electrical_angle);
    
    // 4. 执行空间矢量脉宽调制 (SVPWM) 及逆 Park 变换
    // 根据设定的 Uq (交轴目标电压/用于产生扭矩) 和 Ud (直轴目标电压/用于控制励磁，通常为0)， 
    // 结合当前电角度，计算出三相半桥 (U/V/W) 所需的极快响应的 PWM 占空比。
    // 该路径已经经过定点化(Q15)优化，适合在中断(ISR)内高速运行。
    foc_set_phase_voltage_q15(foc_params.uq_q15, foc_params.ud_q15, foc_params.electrical_angle_u16);
}

void foc_fast_loop_step(void)
{
    foc_fast_loop_step_isr();
}
