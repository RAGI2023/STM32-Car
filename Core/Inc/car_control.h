#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "motor_drive.h"

#define STEER_MOTO_POS		1 
#define MAX_DRIVE_PWM			4000

typedef enum
{
  CarCtrl_STOP        		= 0x00U,    
  CarCtrl_IDLE        		,    
  CarCtrl_START     			,    
} CarCtrl_State_TypeDef;

#pragma pack(4)
typedef struct
{
	float 		speed_KP, speed_KI, speed_KD   ;
} car_config_t ;

typedef struct
{
	int16_t   car_angle ;
	int16_t   car_speed_set[DRIVE_MOTO_NUM] ;
	int16_t   moto_drive[DRIVE_MOTO_NUM] ;
	uint16_t  run_time ;
	uint16_t  run_step ;
}car_ctrl_t;                         

typedef struct
{
	int16_t   car_angle_set ;
	int16_t   car_speed_set[DRIVE_MOTO_NUM] ;
	uint32_t  front_distance_set ;
	uint16_t  run_time_set ; 
}car_plan_t;

void CarCtrl_Start( void );
void CarCtrl_SuperStart1(void);
void CarCtrl_Stop( void );


void CarCtrl_SpeedUp( void );
void CarCtrl_SpeedDown( void );
void CarCtrl_SpeedStop( void );
void CarCtrl_Forward( void );
void CarCtrl_Backward( void );
void CarCtrl_Straight( void );
void CarCtrl_Right( void );
void CarCtrl_Left( void );

void CarCtrl_Init( void );
void CarCtrl_Process( void );

extern __IO CarCtrl_State_TypeDef g_car_ctrl_state ;

#endif
