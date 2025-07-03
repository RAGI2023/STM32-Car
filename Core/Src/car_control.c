#include "car_control.h"
#include "motor_drive.h"
#include "speed_encoder.h"
#include "UltrasonicWave.h"
#include "oled_i2c.h"
#include "ctrl_menu.h"
#include "scope.h"
#include <stdio.h>
#include <string.h>

#define diff_speed 0
#define DIS_THRESH 45000

#define BYPASS_DISTANCE_THRESH 45000  // ??????
#define MAX_BYPASS_CYCLES 3           // ??????

#define LOOP_BEGIN 4
#define LOOP_END 7
#define GOBACK 10
#define MID_FEEDBACK 80
#define ORIGINAL_BACK_TIME 125

uint8_t g_mode3 = 0;
uint8_t bypass_cycle_count = 0;       // ???????
uint8_t avoid_step = 0;
uint8_t AVOIDING = 0;
uint16_t prev_time_set = 0;
__IO CarCtrl_State_TypeDef g_car_ctrl_state = CarCtrl_STOP ;

car_config_t g_CarConfig = 
{
	.speed_KP = 5,
	.speed_KI = 0.1 ,
	.speed_KD = 0
};
car_ctrl_t 	g_CarCtrl;

car_plan_t* g_CarPlan_Ptr;
const int16_t straight_angle = 0;
const uint8_t steer_limit = 90;

car_plan_t car_plan_avoid[] =
{
		{0, {-500, -500}, 0, 40},             // Step 0: 后退
    {55 + straight_angle, {-500, 500}, 0, 140}, // Step 1: Head Left
    {0, {500, 500}, 0, 50},               // Step 2: straight
    {-55 + straight_angle, {500, -500}, 0, 140}, // Step 3: Head Forward ⭐ 检查点1
    
    // 循环段 
		{0, {-500, -500}, 0, 40}, //Step 4:
    {55 + straight_angle, {-500, 500}, 0, 140},  // Step 5: Head Left (循环起点)
    {0, {500, 500}, 0, 80},               // Step 6: Forward
    {-55 + straight_angle, {500, -500}, 0, 80},  // Step 7: Head Forward ⭐ 检查点2
    
    // 最终段 
    {0, {500, 500}, 0, 80},               // Step 8: straight
    {-55 + straight_angle, {500, -500}, 0, 140}, // Step 9: Head Right
    {0, {500, 500}, 0, ORIGINAL_BACK_TIME},               // Step 10: straight back to original path
    {55 + straight_angle, {-500, 500}, 0, 125},  // Step 11: Head Forward
    {straight_angle, {0, 0}, 0, 5},       // Step 12: stop
};

car_plan_t g_CarPlan_Base[] =
{

	//{ -55  , { 0 , 0} , 0 , 100 } ,  		// test steer moto
	
	{ straight_angle  , { 500 ,500} , 0 , 500 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ straight_angle  , { 500 ,500} , 0 , 200},
	//{ straight_angle  , { 500 ,500} , 0 , 360 } ,
	//{ 55+straight_angle  , { 500 , 500} , 0 , 80 } ,
	//{ straight_angle  , { 500 ,500} , 0 , 15} ,
	//{ 55+straight_angle  , { 500 , 500} , 0 , 75 } ,
	
	//{ straight_angle  , { 500 , 500} , 0 , 70 } ,		// run 1.5s with 500mm/s speed straightly
	//	{ 55+straight_angle  , { 500 , 500} , 0 , 80 } ,
	//	{ straight_angle  , { 500 , 500} , 0 , 18} ,
	//{ 45+straight_angle  , { 500 , 500} , 0 , 185 } ,
	//{ straight_angle  , { 500 , 500} , 0 , 150 },
	//{ -55  , { 500 , 500} , 0 , 360 } ,   // turn right 1.1s 
	
	//{ straight_angle  , { 500 , 500} , 0 , 120 } ,		// run 1.2s with 500mm/s speed straightly
	//{ -55  , { 500 , 500} , 0 , 360 } ,		// turn right 1.35s 
	
	//{ straight_angle  , { 500 , 500} , 0 , 180 } ,		// run 1.8s with 500mm/s speed straightly
	{ 0  , { 0   , 0  } , 0 , 0 } ,		// stop
};

car_plan_t g_CarPlan_Supper1[] =
{
	
	{ straight_angle  , { 1100 , 1100} , 55000 , 2000 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{ 55 + straight_angle , { 1100 , 1100} , 0 , 45 } ,		// turn right 1.1s 
	
	//{ straight_angle  , { 1100 , 1100} , 0 , 10 } ,		// run 1s with 50mm/s speed straightly
	{ -55 + straight_angle  , { 1100 ,  1100} , 0 , 130 } ,		// run 1s with 50mm/s speed straightly
	// { straight_angle  , { 500 , 500} , 0 , 20},
	// { -55 + straight_angle  , { 1100 , 1100} , 0 , 220},
	{ 55 + straight_angle  , { 1100 , 1100}, 0 , 40},
	{ straight_angle  , { 1100 , 1100} , 0 , 20},
	{ straight_angle  , { 0   , 0  } , 0 , 0 } ,		// stop
};


car_plan_t g_CarPlan_Supper2[] = 
{
		{ straight_angle  , { 500 ,500} , 0 , 500 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ straight_angle  , { 500 ,500} , 0 , 200},
		{ 0  , { 0   , 0  } , 0 , 0 } ,	
};

/////////////////////////////////////////////////////////////////////////////////
// Menu control
//
/////////////////////////////////////////////////////////////////////////////////

void CarCtrl_PlanSet( void )
{
    if (g_mode3){
        car_plan_t* car_plan_ptr ;

        if (AVOIDING){
            if (avoid_step >= sizeof(car_plan_avoid) / sizeof(car_plan_t)){
                avoid_step = 0;
                AVOIDING = 0;
                bypass_cycle_count = 0;  // 重置绕行计数
				car_plan_avoid[GOBACK].run_time_set	= ORIGINAL_BACK_TIME;						

                g_CarCtrl.run_time = prev_time_set;
                prev_time_set = 0;
                
                printf("Avoid finished! Return to normal path. step = %d %d\n", g_CarCtrl.run_step, g_CarCtrl.run_time);
            }
            
            if (AVOIDING) {
                car_plan_ptr = car_plan_avoid + avoid_step;
            } else {
                car_plan_ptr = g_CarPlan_Ptr + g_CarCtrl.run_step;
            }
        }else{
            car_plan_ptr = g_CarPlan_Ptr + g_CarCtrl.run_step ;
        }
        
        if ( car_plan_ptr->run_time_set == 0 )
        {
            g_car_ctrl_state = CarCtrl_STOP;
            Steer_Moto_Ctrl(STEER_MOTO_POS ,car_plan_ptr->car_angle_set);
            for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
                Drive_Moto_Ctrl( i , 0);
            memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
            bypass_cycle_count = 0;  // 重置计数
            return ;
        }
        
        if ( g_CarCtrl.run_time == 0 )  // load plan 
        {
            g_CarCtrl.run_time ++ ;
            for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
                g_CarCtrl.car_speed_set[i] = car_plan_ptr->car_speed_set[i] ;
            Steer_Moto_Ctrl(STEER_MOTO_POS , car_plan_ptr->car_angle_set );
            
            printf("Load plan: AVOIDING=%d, avoid_step=%d, run_step=%d, bypass_cycles=%d\n", 
                   AVOIDING, avoid_step, g_CarCtrl.run_step, bypass_cycle_count);
        }
        else // execute plan 
        {
            g_CarCtrl.run_time ++ ;
            
            if (g_CarCtrl.run_time >= car_plan_ptr->run_time_set)
            {
                g_CarCtrl.run_time = 0 ;
                
                if (AVOIDING){
                    // ===== 修正后的双检查点逻辑 =====
                    if (avoid_step == 3) {  // 检查点1: Step 3 Head Forward完成后检查
                        printf("Initial Head Forward completed, distance=%d\n", g_ultrawave_data[0].distance);
                        
                        if (g_ultrawave_data[0].distance < BYPASS_DISTANCE_THRESH) {
                            // 障碍太近，进入循环段
                            avoid_step = LOOP_BEGIN;  // 进入Step 4 (Head Left) 开始循环
                            bypass_cycle_count++;
							car_plan_avoid[GOBACK].run_time_set	+= MID_FEEDBACK;						
                            printf("Barrier close! Enter bypass loop cycle %d (step 4)\n", bypass_cycle_count);
                        } else {
                            // 无障碍，跳过循环段直接到最终段
                            avoid_step = LOOP_END + 1;  // 跳转到Step 7 (straight)
                            printf("No barrier! Skip loop, jump to final straight (step 7)\n");
                        }
                    } else if (avoid_step == LOOP_END) {  // 检查点2: Step 6 Head Forward完成后检查 (循环检查点)
                        printf("Loop Head Forward completed, distance=%d\n", g_ultrawave_data[0].distance);
                        
                        if (g_ultrawave_data[0].distance < BYPASS_DISTANCE_THRESH) {
                            // 还有障碍，继续循环 (无次数限制)
                            avoid_step = LOOP_BEGIN;  // 回到Step 4 (Head Left) 继续循环
                            bypass_cycle_count++;
							car_plan_avoid[GOBACK].run_time_set	+= MID_FEEDBACK;						
							
                            printf("Still barrier! Continue loop cycle %d (return to step 4)\n", bypass_cycle_count);
                        } else {
                            // 无障碍，退出循环到最终段
                            avoid_step++;  // 正常进入Step 7 (straight)
                            printf("No barrier! Exit loop, continue to final straight (step %d)\n", avoid_step);
                        }
                    } else {
                        // 其他步骤正常递增
                        avoid_step++;
                        printf("Avoid step updated: %d -> %d\n", avoid_step-1, avoid_step);
                    }
                }else{
                    g_CarCtrl.run_step ++ ;
                    printf("Normal step updated: %d -> %d\n", g_CarCtrl.run_step-1, g_CarCtrl.run_step);
                }
            }

            // 避障触发逻辑
            if (!AVOIDING && g_ultrawave_data[0].distance < DIS_THRESH){
                printf("Obstacle detected! distance=%d, triggering avoidance\n", g_ultrawave_data[0].distance);
                AVOIDING = 1;
                avoid_step = 0;
                bypass_cycle_count = 0;  // 重置绕行计数
                prev_time_set = g_CarCtrl.run_time;
                g_CarCtrl.run_time = 0;
            }
        }
        return ;
    }
    
    //normal mode
    car_plan_t* car_plan_ptr ;
    
    car_plan_ptr = g_CarPlan_Ptr+g_CarCtrl.run_step ;
    
    if ( car_plan_ptr->run_time_set == 0 )
    {
        g_car_ctrl_state = CarCtrl_STOP;
        Steer_Moto_Ctrl(STEER_MOTO_POS ,car_plan_ptr->car_angle_set);
        for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
            Drive_Moto_Ctrl( i , 0);
        memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
        return ;
    }
    
    if ( g_CarCtrl.run_time == 0 )  // load plan 
    {
        g_CarCtrl.run_time ++ ;
        for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
            g_CarCtrl.car_speed_set[i] = car_plan_ptr->car_speed_set[i] ;
        Steer_Moto_Ctrl(STEER_MOTO_POS , car_plan_ptr->car_angle_set );		
    }
    else // execute plan 
    {
        g_CarCtrl.run_time ++ ;
        if ( g_CarCtrl.run_time == car_plan_ptr->run_time_set || 								// plan over
               g_ultrawave_data[0].distance < car_plan_ptr->front_distance_set )   // distance too close
        {
            g_CarCtrl.run_time = 0 ;
            g_CarCtrl.run_step ++ ;			
        }
    }
}

void CarCtrl_Start( void )
{
	g_car_ctrl_state = CarCtrl_IDLE ;
}

void CarCtrl_SuperStart1(void)
{
	g_car_ctrl_state = CarCtrl_IDLE ;
	g_CarPlan_Ptr = g_CarPlan_Supper1;
}

void CarCtrl_SuperStart2(void)
{
	g_car_ctrl_state = CarCtrl_IDLE ;
	g_CarPlan_Ptr = g_CarPlan_Supper2;
	g_mode3 = 1;
}

void CarCtrl_Stop( void )
{
	g_car_ctrl_state = CarCtrl_STOP ;
	Ctrl_Menu_Show();
}

void CarCtrl_SpeedUp( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] += 5 ;
}

void CarCtrl_SpeedDown( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] -= 5 ;
}

void CarCtrl_SpeedStop( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = 0 ;
}

void CarCtrl_Forward( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = abs( g_CarCtrl.car_speed_set[i] );
}

void CarCtrl_Backward( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = -1*abs( g_CarCtrl.car_speed_set[i] );
}

void CarCtrl_Straight( void )
{
	g_CarCtrl.car_angle = 0 ;
	Steer_Moto_Ctrl( STEER_MOTO_POS , g_CarCtrl.car_angle );
}

void CarCtrl_Right( void )
{
	g_CarCtrl.car_angle -- ;
	if ( g_CarCtrl.car_angle < -steer_limit ) g_CarCtrl.car_angle = -steer_limit ;
	Steer_Moto_Ctrl(STEER_MOTO_POS , g_CarCtrl.car_angle );
}

void CarCtrl_Left( void )
{
	g_CarCtrl.car_angle ++ ;
	if ( g_CarCtrl.car_angle > steer_limit ) g_CarCtrl.car_angle = steer_limit ;
	Steer_Moto_Ctrl(STEER_MOTO_POS , g_CarCtrl.car_angle );
}



/////////////////////////////////////////////////////////////////////////////////

void CarCtrl_Init( void )
{
	car_config_t *p_car_cfg = & g_CarConfig ;
	memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
	g_CarPlan_Ptr = g_CarPlan_Base ;
}

void CarCtrl_Speed_PID( )
{
	static int32_t last_speed[DRIVE_MOTO_NUM] = {0,0};
	static int32_t speed_intergrade[DRIVE_MOTO_NUM] = {0,0};
	int32_t speed_error[DRIVE_MOTO_NUM];
	int32_t speed_diff[DRIVE_MOTO_NUM];

	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
	{
		speed_error[i] = g_CarCtrl.car_speed_set[i] - g_speed_encoder[i].speed ;
		speed_intergrade[i] = speed_intergrade[i] + speed_error[i] ; 
		speed_diff[i] = last_speed[i] - g_speed_encoder[i].speed  ;
		last_speed[i] = g_speed_encoder[i].speed ;		
		g_CarCtrl.moto_drive[i] = speed_error[i]*g_CarConfig.speed_KP +
															speed_intergrade[i] * g_CarConfig.speed_KI +
															speed_diff[i] * g_CarConfig.speed_KD  ;
		
		//Drive_Moto_Ctrl( i , g_CarCtrl.moto_drive[i] );
	}
	
	Drive_Moto_Ctrl( 0 , g_CarCtrl.moto_drive[0] );
	Drive_Moto_Ctrl( 1 , -g_CarCtrl.moto_drive[1] );	
}



void CarCtrl_Show( void ) 
{
	static uint8_t  index = 0 ;
	static int32_t  speed[DRIVE_MOTO_NUM] = { 0 , 0 } ;
	static int32_t  pwm[DRIVE_MOTO_NUM] = { 0 , 0 } ;
	uint8_t 				buf[17];
	
	for (int i = 0 ; i < DRIVE_MOTO_NUM ; i++) 
	{	
		speed[i] += g_speed_encoder[ i ].speed ;
		pwm[i] += g_CarCtrl.moto_drive[ i ] ;
	}
	
	if ( index < 9 ) 
	{
		index++;
	}
	else
	{
		index = 0 ;
		speed[0] = speed[0] / 10 ;
		speed[1] = speed[1] / 10 ;
		pwm[0] = pwm[0] / 10 ;
		pwm[1] = pwm[1] / 10 ;
		sprintf( buf , "L:%d R:%d" , speed[0] , speed[1] );
		OLED_ShowAscii( 0,0, buf , 16 ,0 );
		
		sprintf( buf , "L:%d R:%d" , pwm[0] , pwm[1] );
		OLED_ShowAscii( 1,0, buf , 16 ,0 );
		
		speed[0] = 0 ;
		speed[1] = 0 ;		
		pwm[0] = 0 ;
		pwm[1] = 0 ;
	}
}

// ?? car_control.c ??

// ...
void CarCtrl_Process( void )
{
	static int scope_send_counter = 0;

	if ( g_car_ctrl_state == CarCtrl_STOP ) return ;
	if ( g_car_ctrl_state == CarCtrl_START ) 
	{
		g_car_ctrl_state = CarCtrl_IDLE ;
		Speed_Calculate();
		CarCtrl_Speed_PID();
	
		scope_send_counter++;
		if (scope_send_counter >= 10) // ????10Hz????????
		{
			scope_send_counter = 0;
			printf("Now avoiding status :%d, avoid_step = %d\n", AVOIDING, avoid_step);
            // ======================================================================
            // == ???????????????????????
            // ======================================================================
			// Scope_Send4Floats(
      //           (float)g_CarCtrl.car_speed_set[0],  // ???1: ??????????
      //           (float)g_speed_encoder[0].speed,    // ???2: ??????????
      //           (float)g_ultrawave_data[0].distance / 1000.0,  // ???3: ??????????
      //          (float)g_ultrawave_data[1].distance / 1000.0    // ???4: ??????????
      //       );
            // ======================================================================
		}
		
		CarCtrl_PlanSet();
	}
}