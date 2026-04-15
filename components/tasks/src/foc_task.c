#include "foc_task.h"

#include "foc_control.h"
#include "foc_output.h"
#include "mt6701.h"

static uint16_t s_zero_electric_angle_u16 = 0U;

void foc_task_init(void)
{
    // 零电角在运行时不会频繁变化，
    // 所以提前缓存成 u16 角度格式，减少快环里的浮点换算。
    s_zero_electric_angle_u16 = foc_float_to_angle_u16(foc_params.zero_electric_angle);
}

void IRAM_ATTR foc_fast_loop_step_isr(void)
{
    // 快环中断只保留最核心的实时路径：
    // 1. 读取机械角
    // 2. 计算电角
    // 3. 下发三相电压
    if (!foc_enabled || !foc_started || !foc_calibrated) {
        return;
    }

    uint16_t shaft_raw = 0U;
    if (MT6701_ReadAngle(&shaft_raw, NULL, NULL) != ESP_OK) {
        return;
    }

    // MT6701 返回 14bit 原始角度，这里统一扩展为 16bit 环形角度。
    foc_params.shaft_angle_u16 = (uint16_t)(shaft_raw << 2);

    // 电角 = 机械角 * 极对数 * 方向 - 零电角
    foc_params.electrical_angle_u16 =
        (uint16_t)(((int32_t)foc_params.shaft_angle_u16 * foc_params.pole_pairs *
                    ((foc_params.sensor_direction >= 0) ? 1 : -1)) -
                   s_zero_electric_angle_u16);

    foc_set_phase_voltage_q15(foc_params.uq_q15, foc_params.ud_q15, foc_params.electrical_angle_u16);
}
