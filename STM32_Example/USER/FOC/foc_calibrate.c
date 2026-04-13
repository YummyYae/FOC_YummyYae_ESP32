#include "foc_calibrate.h"
#include "flash_storage.h"
#include "foc_utils.h"
#include "tim.h"
#include "adc.h"
#include "mt6826s.h"
#include "foc_log.h"
#include <math.h>

#define M_PI 3.14159265358979323846f


// 缁熶竴FOC鍙傛暟缁撴瀯浣?
FOC_Parameters foc_params;

uint8_t foc_enabled = 1;
uint8_t foc_started = 0;
uint8_t foc_calibrated = 0;



void FOC_Parameters_Init(FOC_Parameters *params) {
    if (!params) return;
    params->iu = params->iv = params->iw = 0;
    params->iu_offset = params->iv_offset = params->iw_offset = 0;
    params->pwm_u = params->pwm_v = params->pwm_w = 0;
    params->uq = params->ud = 0;
    params->iq = params->id = 0;
    params->i_alpha = params->i_beta = 0;
    params->v_alpha = params->v_beta = 0;
    params->voltage_power_supply = 12.0f;
    params->pole_pairs = 7;
    params->sensor_direction = 1;
    params->zero_electric_angle = 0.0f;
    params->phase_resistance = 0;
}

void setTargetVotage(float target_q,float target_d)
{
	foc_params.uq = target_q;
	foc_params.ud = target_d;
}
void setTargetI (float targeti_q,float targeti_d)
{
	foc_params.target_iq = targeti_q;
	foc_params.target_id = targeti_d;
}
void Clarke_Transf(float Current_abc_temp[3],float Current_alpha_beta_temp[2])
{
    foc_params.i_alpha = (Current_abc_temp[0] - (Current_abc_temp[1] + Current_abc_temp[2]) * 0.5F) * 2.0F / 3.0F;
    foc_params.i_beta  = (Current_abc_temp[1] - Current_abc_temp[2]) * 0.866025388F * 2.0F / 3.0F;
    if (Current_alpha_beta_temp) {
        Current_alpha_beta_temp[0] = foc_params.i_alpha;
        Current_alpha_beta_temp[1] = foc_params.i_beta;
    }
}

void Park_Transf(float Current_alpha_beta_temp[2],float angle_el,float current_dq_temp[2])
{
    foc_params.id = Current_alpha_beta_temp[0] *_cos(angle_el) + Current_alpha_beta_temp[1] *_sin(angle_el);
    foc_params.iq = -Current_alpha_beta_temp[0] * _sin(angle_el) + Current_alpha_beta_temp[1] * _cos(angle_el);
    if (current_dq_temp) {
        current_dq_temp[0] = foc_params.id;
        current_dq_temp[1] = foc_params.iq;
    }
}

#define PWM_Period 2124

// 悴WM占毡龋实SVPWM占矢
// Uq: q压Ud: d压angle_el: 嵌
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;      // 杈撳嚭鐢靛帇骞呭€?
    uint32_t sector; // SVPWM鎵囧尯
    float T0,T1,T2;  // SVPWM瀹氭椂
    float Ta,Tb,Tc;  // 涓夌浉鍗犵┖姣?
    // 璁＄畻Uq Ud鍚堟垚骞呭€?
    Uout = _sqrt(Ud*Ud + Uq*Uq) / foc_params.voltage_power_supply;
      angle_el = _normalizeAngle(angle_el + _atan2(Uq, Ud));
      if(Uout > 0.577f) Uout = 0.577f;
      if(Uout < -0.577f) Uout = -0.577f;
    // SVPWM
    sector = (uint32_t)(angle_el / (M_PI / 3.0f)) + 1;
    if (sector < 1) sector = 1; // 防止越界死区
    if (sector > 6) sector = 6;
    float angle_sector = angle_el - (float)(sector - 1) * (M_PI / 3.0f);
    T1 = Uout * _sin((M_PI / 3.0f) - angle_sector);
    T2 = Uout * _sin(angle_sector);
    T0 = 1.0f - T1 - T2;
    // 璁＄畻涓夌浉PWM鍗犳瘮
    switch(sector) {
        case 1:
            Ta = T1 + T2 + T0/2;
            Tb = T2 + T0/2;
            Tc = T0/2;
            break;
        case 2:
            Ta = T1 +  T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T0/2;
            break;
        case 3:
            Ta = T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T2 + T0/2;
            break;
        case 4:
            Ta = T0/2;
            Tb = T1+ T0/2;
            Tc = T1 + T2 + T0/2;
            break;
        case 5:
            Ta = T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T2 + T0/2;
            break;
        case 6:
            Ta = T1 + T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T0/2;
            break;
        default:  // 鐘舵€佸叏鏃?
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    foc_params.pwm_u = Ta;
    foc_params.pwm_v = Tb;
    foc_params.pwm_w = Tc;
    
    // 鍐欏叆瀹氭椂鍣≒WM鍗犵┖姣旓紝瀹炵幇鐢靛帇杈撳嚭
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Ta*PWM_Period);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,Tb*PWM_Period);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,Tc*PWM_Period);
}


// 鍋囪浣跨敤鐨勯檺骞呰緟鍔╁嚱鏁帮紙鏆傛湭浣跨敤锛屽睆钄戒互娑堥櫎璀﹀憡锛?
#if 0
static float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
#endif

// 鎮ㄦ湡鏈涚殑鏁版嵁瀛樻斁浣嶇疆鏁扮粍锛堝彈闄愪簬STM32纭欢鐗规€э紝娉ㄥ叆缁勬棤娉曠洿鎺ヤ娇鐢―MA纭欢鎼繍锛屼絾鎴戝凡鍦ㄨ繖涓嚱鏁颁腑鑷姩涓烘偍瀹屾垚浜嗗悜杩欎釜鏁扮粍鐨勨€滄惉杩愨€濓級
uint32_t adc1_dma_buf[3];

// 鏍规嵁10m惟閲囨牱鐢甸樆鍙?鍊嶅鐩婏紝ADC娴嬪緱鐢靛帇V = I * 0.010 * 2 + V_offset
// 鐢垫祦 I = (V - V_offset) / 0.020

void update_current_sensor(float electrical_angle) {
    // 渚濇墭鐢ㄦ埛宸茬粡鍦ㄥ簳灞傞厤濂界殑鐗规畩纭欢閲囨牱瑙﹀彂浣嶇疆锛圱IM1 TRGO2锛夛紝鐩存帴璇诲彇鏈€鏂扮殑娉ㄥ叆缁勬暟鎹?
    // 灏嗗叾鑷姩瀛樺叆鎮ㄩ渶瑕佺殑鐙珛鏁扮粍 adc1_dma_buf 涓?
    adc1_dma_buf[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    adc1_dma_buf[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    adc1_dma_buf[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

    // 杞崲涓虹湡瀹炲紩鑴氱數鍘?鍋囪3.3V鍙傝€?
    float vol_u = adc1_dma_buf[0] * 0.0008058601f;
    float vol_v = adc1_dma_buf[1] * 0.0008058601f;
    float vol_w = adc1_dma_buf[2] * 0.0008058601f;
    // 杩樺師璁＄畻鍚勭浉鐢垫祦
    foc_params.iu = (foc_params.iu_offset - vol_u) * 2.0f; // 修正回真实硬件运放增益 50x (0.01R * 50 = 0.5V/A)
    foc_params.iv = (foc_params.iv_offset - vol_v) * 2.0f; // 修正回真实硬件运放增益 50x (0.01R * 50 = 0.5V/A)
    foc_params.iw = (foc_params.iw_offset - vol_w) * 2.0f; // 修正回真实硬件运放增益 50x (0.01R * 50 = 0.5V/A)

    // 鏍规嵁璇诲彇鐨勭浉鐢垫祦杩涜鐩哥數娴佸彉鎹㈣绠?(鍏嬫媺鍏嬩笌甯曞厠鍙樻崲)
    // 1. 鍏嬫媺鍏嬪彉鎹?
    foc_params.i_alpha = foc_params.iu;
    foc_params.i_beta = (foc_params.iu + 2.0f * foc_params.iv) * _1_SQRT3;

    // 2. 甯曞厠鍙樻崲
    float cos_angle = _cos(electrical_angle);
    float sin_angle = _sin(electrical_angle);
    foc_params.iq = foc_params.i_beta * cos_angle - foc_params.i_alpha * sin_angle;
    foc_params.id = foc_params.i_beta * sin_angle + foc_params.i_alpha * cos_angle;
}

// 鍒濆鍖栵紝涓婄數鏃惰皟鐢?
void FOC_Calibrate_Init(void) {
    FOC_Parameters_Init(&foc_params);
    // ==== 1. ADC鍚姩鍓嶅姟蹇呭厛杩涜鏍″噯 (ADEN=0)锛屽鏋滃湪Start涔嬪悗鍐嶈皟鍒欏叏閮ㄥけ鏁?====
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    
    // ==== 鏆村姏閲嶅啓 ADC 娉ㄥ叆缁勯厤缃互琛ユ晳 CubeMX 婕忛厤鍜岄敊閰?====
    // 寮哄埗涓?ADC1 閰嶇疆 Rank1~3 鍒嗗埆瀵瑰簲 PA1(IN2), PA2(IN3), PA3(IN4)锛屽苟缁х画鎸傝浇鍦?TIM1_TRGO2 瑙﹀彂涓?
    ADC_InjectionConfTypeDef sConfigInjected = {0};
    sConfigInjected.InjectedNbrOfConversion = 3;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.QueueInjectedContext = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode = DISABLE;
    
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    
    // Rank 1 -> PA1(IN2)
    sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5; // 鎷夐暱閲囨牱鏃堕棿闃叉姈
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);
    
// Rank 2 -> PA3(IN4) 璇诲彇Phase V
    sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    // Rank 3 -> PA2(IN3) 璇诲彇Phase W
    sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    // 瀵逛簬 ADC2 (娴嬬數鍘嬬殑 PA4/IN17)锛屾垜浠璧嬩簣鍏剁函杞欢瑙﹀彂鏉冮檺
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START; // 闈炲父閲嶈锛?
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_17;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    // 寮€鍚粦瀹氬湪TIM1_TRGO2纭欢瑙﹀彂涓婄殑ADC1娉ㄥ叆缁?(鍑嗗鎺ユ敹PWM瑙﹀彂淇″彿)
    HAL_ADCEx_InjectedStart(&hadc1);

    // ==== 姣忔涓婄數锛屽己鍒舵牎鍑咥DC鐩哥數娴侀浂鐐瑰亸缃強鎬荤數婧愮數鍘?====
    // 璁＄畻鏃犵數娴佺姸鎬佷笅鐨勪腑鐐瑰亸缃數鍘嬪強渚涚數鐢靛帇锛屾鎿嶄綔鏃犺鏄惁鏈塅lash璁板綍閮藉繀椤诲仛
    setPhaseVoltage(0, 0, 0); // 涓嬪彂鍏?鐢靛帇鑾峰彇绾噣鍋忓樊
    HAL_Delay(300);
    
    HAL_ADC_Start(&hadc2); // 鍏堝惎鐢ˋDC2绋冲Ε
    
    float sum_u = 0, sum_v = 0, sum_w = 0, sum_vol = 0;
    for(int i = 0; i < 200; ++i) {
        HAL_Delay(1);
        
        // 杞欢瑙﹀彂ADC2鑾峰彇鐢垫簮鍒嗗帇
        HAL_ADCEx_InjectedStart(&hadc2);
        if(HAL_ADCEx_InjectedPollForConversion(&hadc2, 10) == HAL_OK) {
            uint32_t val_vol = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            sum_vol += (val_vol / 4095.0f) * 3.3f * 8.5f;
        }
        
        // TIM1姝ｅ湪浜х敓鍗犵┖姣旇Е鍙慉DC1杞崲锛岀洿鎺ヨ鍙栦笁鐩哥浉鐢垫祦
        uint32_t val_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint32_t val_v = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        uint32_t val_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
        
        sum_u += (val_u / 4095.0f) * 3.3f;
        sum_v += (val_v / 4095.0f) * 3.3f;
        sum_w += (val_w / 4095.0f) * 3.3f;
    }
    
    HAL_ADCEx_InjectedStop(&hadc2);
    HAL_ADC_Stop(&hadc2);
    
    foc_params.voltage_power_supply = sum_vol / 200.0f + 0.04f ;
    // 涓囦竴ADC璇诲彇寮傚父瀵艰嚧杩囦綆锛屾彁渚涗竴涓繚搴?
    if(foc_params.voltage_power_supply < 5.0f) {
        foc_params.voltage_power_supply = 12.0f;
    }
    
    foc_params.iu_offset = sum_u / 200.0f;
    foc_params.iv_offset = sum_v / 200.0f;
    foc_params.iw_offset = sum_w / 200.0f;
    
    printf("Power Supply Voltage Calibrated: %.2fV\r\n", foc_params.voltage_power_supply);

    FlashStorage_Init();
    
    if (g_motor_data.storage_status == STORAGE_STATUS_CALIBRATED ||
        g_motor_data.storage_status == STORAGE_STATUS_SAVED) {
        // 宸叉湁鏍″噯鎴栦繚瀛樿繃鐨勬暟鎹紝鍙互灏嗗叾澶嶅師鍒癋OC鍙傛暟缁撴瀯浣?
        foc_params.zero_electric_angle = g_motor_data.base_calibrate_data.zero_electric_angle;
        foc_params.sensor_direction = g_motor_data.base_calibrate_data.encoder_direction ? 1 : -1;
        foc_params.phase_resistance = g_motor_data.base_calibrate_data.phase_resistance;
        foc_params.pole_pairs = g_motor_data.base_calibrate_data.pole_pairs;
        // 鍔ㄦ€佽鐩栨渶澶?鏈€灏忕數鍘嬮檺鍒讹紙鏍规嵁鏈涓婄數瀹炴祴鍒嗗帇锛?
        g_motor_data.limit_max = foc_params.voltage_power_supply;
        g_motor_data.limit_min = -foc_params.voltage_power_supply;
        foc_calibrated = 1;
        printf("Flash data LOADED successfully.\r\n");
        PrintCalibrationResult(); // 鎶婂彇鍑烘潵鐨勫弬鏁颁篃鍚屾牱鎵撳嵃涓€浠藉嚭鏉ユ牳瀵?
    } else {
        printf("Flash empty or corrupted, STARTING calibration...\r\n");
        FOC_Calibrate_Run();
    }
}

// 鎵ц鏍″噯锛屽綋杩涜闇€瑕侀噸鏍″噯鐢垫満鏃惰璋冪敤
void FOC_Calibrate_Run(void) {
    if (!foc_enabled) return;
    if (foc_started) return;
    foc_calibrated = 0;
    
    /*2.娴嬮噺鐩哥數闃?纭畾鍚庣画鏍″噯浣跨敤鐢靛帇*/
    // 鏆傛椂鐣ヨ繃鐩哥數闃绘祴閲忥紝璁剧疆涓€涓浐瀹氱殑鏍″噯鐢靛帇锛屾瘮濡?1.5V
    foc_params.phase_resistance = 0.0f;
    const float voltage_align = 1.5f; 
    
    /*3.寮哄埗鎷栧姩鐢垫満鐩村埌瀹屾暣鏈烘涓€鍦堬紝璁＄畻鏋佸鏁板苟鍚屾鏀堕泦鍚勭數鍛ㄦ湡鏈熬瀵瑰簲闆剁偣*/
    // 閿佸畾鍒濆浣嶇疆涓?鐢佃搴?(_3PI_2涓篞杞撮浂浣?
    setPhaseVoltage(voltage_align, 0, _3PI_2); 
    HAL_Delay(500);
    
    float sz = 0.0f, cz = 0.0f;
    for(int i = 0; i < 100; ++i) {
        float a = MT6826S_GetAngle();
        sz += sin(a);
        cz += cos(a);
        HAL_Delay(1);
    }
    float start_angle = atan2(sz, cz);
    if(start_angle < 0) start_angle += 2.0f * M_PI;
    
    float recorded_mech_angles[100];
    recorded_mech_angles[0] = start_angle;
    
    float prev_angle = start_angle;
    float total_moved = 0.0f;
    int cycle_count = 0;
    
    printf("Calibration: Dragging motor until 1 full mechanical rev...\r\n");
    
    // 鏈€澶氭嫋鍔ㄤ笉鍒?00涓數鍛ㄦ湡浣滀负淇濇姢涓婇檺
    while (cycle_count < 99) {
        cycle_count++;
        
        // 姝ｅ悜鎷栧姩鍒氬ソ涓€涓數鍛ㄦ湡
        for (int j = 1; j <= 125; ++j) {
            float angle = _3PI_2 + 2.0f * M_PI * j / 125.0f;
            setPhaseVoltage(voltage_align, 0, angle);
            HAL_Delay(1);
        }
        // 淇濊瘉鏈數鍛ㄦ湡鎷栧姩瀹屾瘯鍚庣簿纭攣瀹氬湪涓嬩釜 0 鐢佃搴︿綅
        setPhaseVoltage(voltage_align, 0, _3PI_2);
        HAL_Delay(300);
        
        // 閲囨牱褰撳墠鏈烘瑙掑害
        sz = 0.0f; cz = 0.0f;
        for(int i = 0; i < 100; ++i) {
            float a = MT6826S_GetAngle();
            sz += sin(a);
            cz += cos(a);
            HAL_Delay(1);
        }
        float current_angle = atan2(sz, cz);
        if(current_angle < 0) current_angle += 2.0f * M_PI;
        
        recorded_mech_angles[cycle_count] = current_angle;
        
        // 鏃犺鍦堟暟鐨勭煭璺濈鏈烘鍙樺寲閲?
        float delta = current_angle - prev_angle;
        if (delta > M_PI) delta -= 2.0f * M_PI;
        else if (delta < -M_PI) delta += 2.0f * M_PI;
        
        total_moved += delta;
        prev_angle = current_angle;
        
        // 缁撴潫鏉′欢锛氱疮绉祴寰楃殑鏈烘浣嶇Щ閲忚揪鍒颁簡鎺ヨ繎 2PI (鐢变簬涓€鍦堟渶澶氫篃鍙法瓒婂崐涓猟elta宸紓锛岄鐣?0.5*delta 浣滃啑浣?
        if (fabs(total_moved) >= 2.0f * M_PI - fabs(delta) * 0.5f) {
            break;
        }
    }
    
    setPhaseVoltage(0, 0, 0);
    HAL_Delay(200);
    
    if (fabs(total_moved) < 0.5f) {
        printf("Calibration Warning: Motor blocked or not moving properly!\r\n");
    } else {
        foc_params.sensor_direction = (total_moved > 0) ? 1 : -1;
        foc_params.pole_pairs = cycle_count; // 杞繃 1 鏈烘鍛ㄦ湡鐨勭數鍛ㄦ湡鏁板氨鏄瀬瀵规暟!
    }
    
    uint8_t encoder_direction = (foc_params.sensor_direction == 1) ? 1 : 0;
    
    /*4.璁＄畻鐢佃搴﹂浂鐐瑰钩鍧囧亸宸?*/
    float z_sz = 0.0f, z_cz = 0.0f;
    
    // 鎴戜滑浠?i=0 (鍗宠捣濮嬮攣瀹氱偣) 閬嶅巻鍒?cycle_count (鎭板ソ灏辨槸鏋佸鏁? 
    // 鑾峰彇姣忎釜璁板綍鐨勭數瑙?浣嶅搴旂殑鏈烘鍘熷瑙掑害锛屽悎骞剁畻娉曚互骞冲潎鎺夌閾佸畨瑁呭亸蹇?
    for (int i = 0; i <= foc_params.pole_pairs; ++i) {
        float current_zero_offset = _normalizeAngle((float)(foc_params.sensor_direction * recorded_mech_angles[i] * foc_params.pole_pairs));
        z_sz += sin(current_zero_offset);
        z_cz += cos(current_zero_offset);
    }
    
    // 鍊熷姪 atan2 鑾峰彇瀹岀編骞虫粦鐨勫叏鏈烘鍦堢殑瀹氬瓙闆跺亸骞冲潎
    foc_params.zero_electric_angle = atan2(z_sz, z_cz);
    if(foc_params.zero_electric_angle < 0) foc_params.zero_electric_angle += 2.0f * M_PI;
    
    foc_calibrated = 1;
    
    // == 淇濆瓨鍒癋lash ==
    g_motor_data.base_calibrate_data.phase_resistance = foc_params.phase_resistance;
    g_motor_data.base_calibrate_data.voltage_align = voltage_align;
    g_motor_data.base_calibrate_data.encoder_direction = encoder_direction;
    g_motor_data.base_calibrate_data.zero_electric_angle = foc_params.zero_electric_angle;
    g_motor_data.base_calibrate_data.pole_pairs = foc_params.pole_pairs;
    
    g_motor_data.zero_pos = 0.0f;
    g_motor_data.pid_kp = 1.0f;
    g_motor_data.pid_ki = 0.0f;
    g_motor_data.pid_kd = 0.0f;
    g_motor_data.limit_max = foc_params.voltage_power_supply;
    g_motor_data.limit_min = -foc_params.voltage_power_supply;
    g_motor_data.plug_detected = 1;
    
    foc_params.sensor_direction = encoder_direction ? 1 : -1;
    
    g_motor_data.storage_status = STORAGE_STATUS_CALIBRATED;
    FlashStorage_SaveData();

    // 5. 鎵撳嵃鍒氬垰鑾峰彇骞跺叆妗ｇ殑鍚勯」璁＄畻鍙傛暟
    PrintCalibrationResult();
}


