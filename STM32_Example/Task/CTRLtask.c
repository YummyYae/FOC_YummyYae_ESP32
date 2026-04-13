#include "CTRLtask.h"
#include "foc_calibrate.h"
#include "pid.h"
#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ================== 控制模式选择 ==================
#define MODE_NORMAL         0   // 正常模式
#define MODE_WAVEFORM       1   // 测试波形
#define MODE_RATCHET_LIMIT  2   // 模拟有限位拨轮 (+-1.5圈机械限位死区)
#define MODE_SPRING         3   // 模拟扭簧/发条 (越转越紧，松手弹回起点)
#define MODE_DAMPED         4   // 模拟液压阻尼旋钮 (无极无顿挫，但像搅动蜂蜜一样费力)

// [玩法开关] 在这里切换你要体验的模式！
#define CURRENT_CTRL_MODE   MODE_RATCHET_LIMIT

// --- 若选择 MODE_WAVEFORM 的附属设定 ---
#define WAVE_NONE     0
#define WAVE_SINE     1
#define WAVE_TRIANGLE 2
#define WAVE_SQUARE   3
#define TARGET_WAVEFORM WAVE_SINE
// =================================================

void CTRL_Task_Init(void) {
    foc_params.motor_speed = 0.0f;
    foc_params.target_speed = 0.0f; 
    foc_params.motor_position = 0.0f;
    foc_params.target_position = 0.0f; 
}

// === 功能核心 1：状态位移观测器 ===
void Update_Motor_State(void) {
    static float prev_shaft_angle = 0.0f;
    static bool first_run = true;
    
    float dt = 0.004f; // 250Hz -> 4ms
    float current_angle = foc_params.shaft_angle;
    
    if (first_run) {
        prev_shaft_angle = current_angle;
        first_run = false;
    }

    float delta_angle = current_angle - prev_shaft_angle;

    // 处理过零跳变问题 (+/- M_PI)
    if (delta_angle > M_PI) delta_angle -= 2.0f * M_PI;
    else if (delta_angle < -M_PI) delta_angle += 2.0f * M_PI;

    foc_params.motor_position += delta_angle;

    float raw_speed = delta_angle / dt;

    float alpha = 0.05f;
    foc_params.motor_speed = alpha * raw_speed + (1.0f - alpha) * foc_params.motor_speed;
    prev_shaft_angle = current_angle;
}

// === 测试玩法 2：波形发生器 ===
void Generate_Target_Position(void) {
#if CURRENT_CTRL_MODE == MODE_WAVEFORM && TARGET_WAVEFORM != WAVE_NONE
    static float time_t = 0.0f;
    float dt = 0.004f; // 250Hz 时基
    time_t += dt;

    float amplitude = 3.14159f * 2.0f; // 振幅：一圈 (2*PI Rad)
    float frequency = 0.5f;            // 频率：0.5Hz
    
    #if TARGET_WAVEFORM == WAVE_SINE
        foc_params.target_position = amplitude * sinf(2.0f * M_PI * frequency * time_t);
    #elif TARGET_WAVEFORM == WAVE_TRIANGLE
        float period = 1.0f / frequency;
        float phase = time_t / period;
        phase = phase - (int)phase;
        if (phase < 0.25f) foc_params.target_position = amplitude * (phase * 4.0f);
        else if (phase < 0.75f) foc_params.target_position = amplitude * (2.0f - phase * 4.0f);
        else foc_params.target_position = amplitude * (phase * 4.0f - 4.0f);
    #elif TARGET_WAVEFORM == WAVE_SQUARE
        float period = 1.0f / frequency;
        float phase = time_t / period;
        phase = phase - (int)phase; 
        foc_params.target_position = (phase < 0.5f) ? amplitude : -amplitude;
    #endif
#endif
}

// === 测试玩法 3：带物理边界的拨轮产生器 (正反1.5圈) ===
void Simulate_Ratchet_Wheel_Limit(void) {
    // 设定一圈有几个齿轮段位
    const float num_detents = 24.0f; 
    float step_angle = (2.0f * M_PI) / num_detents;
    
    // 物理限位：正反 1.5 圈，共计 3*PI
    const float limit_angle = 1.5f * 2.0f * M_PI; 
    
    float current_pos = foc_params.motor_position;
    
    if (current_pos >= limit_angle) {
        // 超出边界，像撞墙一样死死拉在这个角度
        foc_params.target_position = limit_angle;
    } else if (current_pos <= -limit_angle) {
        // 反向超出边界
        foc_params.target_position = -limit_angle;
    } else {
        // 在边界内，正常寻找最近的齿穴吸附
        float nearest_detent = roundf(current_pos / step_angle) * step_angle;
        foc_params.target_position = nearest_detent;
    }
}

// === 测试玩法 4：扭簧/发条效果 ===
void Simulate_Spring(void) {
    // 目标永远是起点。依靠你设置的位置环 PID，
    // 拧得越偏离0，位置环产生的恢复力矩越大，松手就会“砰”地弹回去！
    foc_params.target_position = 0.0f;
}

// === 测试玩法 5：液压门/重型阻尼旋钮 ===
void Simulate_Damped_Knob(void) {
    // 让目标位置一直紧跟当前位置，彻底废除位置环产生的弹簧力
    foc_params.target_position = foc_params.motor_position;
    
    // 把目标速度设为0，此时你的手转得越快，速度环产生的反抗电流（刹车阻尼）就越大！
    // 完美模拟搅动浓稠蜂蜜的流体阻尼质感。
    foc_params.target_speed = 0.0f;
}

// === 主控制调度任务 ===
void CTRL_Task(void) {
    // 1. 获取解算最新位移与速度
    Update_Motor_State();

    // 2. 根据宏定义切换不同的趣味玩法路由
#if CURRENT_CTRL_MODE == MODE_WAVEFORM
    Generate_Target_Position();
#elif CURRENT_CTRL_MODE == MODE_RATCHET_LIMIT
    Simulate_Ratchet_Wheel_Limit();
#elif CURRENT_CTRL_MODE == MODE_SPRING
    Simulate_Spring();
#elif CURRENT_CTRL_MODE == MODE_DAMPED
    Simulate_Damped_Knob();
#endif

    // 3. 执行常规三闭环结算路线
    foc_params.target_speed = PID_Calc(&pid_position, foc_params.motor_position, foc_params.target_position);
    
    // 如果是阻尼模式，因为我们已经自己锁死了 target_speed 为 0，就跳过位置环的速度赋给
#if CURRENT_CTRL_MODE == MODE_DAMPED
    foc_params.target_speed = 0.0f; 
#endif

    foc_params.target_iq = -PID_Calc(&pid_speed, foc_params.motor_speed, foc_params.target_speed);
}
