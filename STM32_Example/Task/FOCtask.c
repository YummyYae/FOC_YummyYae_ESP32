#include "mt6826s.h"
#include "foc_utils.h"
#include "foc_calibrate.h"
#include "pid.h"
#include "main.h"

// 1kHz / 5kHz 频率被 TIM6 调用的核心 FOC 任务计算
void FOC_Task(void)
{
    static uint32_t loop_counter = 0;
    loop_counter++;
    if (loop_counter >= 5000) {
        loop_counter = 0;
        // 如果 FOC_Task 是 5kHz 运行，则每 5000 次执行恰好 1 秒
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    }

    // 1. 读取绝对角度传感器的机械真实位置 (如 MT6826)
    foc_params.shaft_angle = MT6826S_GetAngle();
    
    // 2. 将机械角转换为当前的电角度并加上零点偏置
    foc_params.electrical_angle = _normalizeAngle((float)(foc_params.sensor_direction * foc_params.shaft_angle * foc_params.pole_pairs) - foc_params.zero_electric_angle);

    // 实时读取最新由特殊触发（如TRGO2）采集到的三相相电流，并内部完成克拉克与帕克变换计算
    update_current_sensor(foc_params.electrical_angle);

    // 3. 运行闭环FOC环算法
    FOC_Current_PID_Update(foc_params.target_id, foc_params.target_iq);
    setPhaseVoltage(foc_params.uq, foc_params.ud, foc_params.electrical_angle);
}
