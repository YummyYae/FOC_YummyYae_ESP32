/**
 * @brief   Encoder MT6826S Driver C Version
 * @details Converted from C++ class
 */
#ifndef ENCODER_DRIVER_MT6826S_H
#define ENCODER_DRIVER_MT6826S_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void MT6826S_Init(void);
void MT6826S_Enable(void);
void MT6826S_Disable(void);
float MT6826S_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif //ENCODER_DRIVER_MT6826S_H
