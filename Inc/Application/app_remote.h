/*
 *  Project      : Polaris
 * 
 *  file         : app_remote.h
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 13:20:47
 */


#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif


#include "periph_remote.h"
#include "periph_servo.h"
#include "alg_math.h"

typedef struct {
    uint8_t pending;
    float yaw_angle_offset;
} Remote_RemoteControlTypeDef;

void Remote_Task(void const * argument);
void Remote_RemotrControlInit(void);
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr(void);
void Remote_ControlCom(void);
void Remote_MouseShooterModeSet(void);
void Remote_RemoteShooterModeSet(void);
void Remote_RemoteProcess(void);
void Remote_KeyMouseProcess(void);
uint8_t Is_Key_Triggered(uint8_t current_state, uint8_t *last_state, uint32_t *last_tick, uint32_t cool_time);

extern float vef_pitch_ref;
extern int count_cqie;
extern uint8_t q_mode;
#endif

#ifdef __cplusplus
}
#endif
