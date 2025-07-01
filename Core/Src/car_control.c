#include "car_control.h"
#include "motor_drive.h"
#include "speed_encoder.h"
#include "UltrasonicWave.h"
#include "oled_i2c.h"
#include "ctrl_menu.h"
#include "scope.h"

__IO CarCtrl_State_TypeDef g_car_ctrl_state = CarCtrl_STOP ;

car_config_t g_CarConfig = 
{
	.speed_KP = 5,
	.speed_KI = 0.1 ,
	.speed_KD = 0
};
car_ctrl_t 	g_CarCtrl;

car_plan_t* g_CarPlan_Ptr;

car_plan_t g_CarPlan_Base[] =
{
	//{ 55  , { 0 , 0} , 0 , 100 } ,   		// test steer moto
	//{ -55  , { 0 , 0} , 0 , 100 } ,  		// test steer moto
	
	{ 0  , { 500 , 500} , 0 , 200 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 0  , { 500 , 500} , 0 , 150 } ,		// run 1.5s with 500mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 110 } ,   // turn right 1.1s 
	
	{ 0  , { 500 , 500} , 0 , 120 } ,		// run 1.2s with 500mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 135 } ,		// turn right 1.35s 
	
	{ 0  , { 500 , 500} , 0 , 180 } ,		// run 1.8s with 500mm/s speed straightly
	{ 0  , { 0   , 0  } , 0 , 0 } ,		// stop
};

car_plan_t g_CarPlan_Supper[] =
{
	{ 55  , { 0 , 0} , 0 , 100 } ,   		// test sreer moto
	{ -55  , { 0 , 0} , 0 , 100 } ,  		// test sreer moto
	
	{ 17  , { 500 , 500} , 20000 , 100 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 17  , { 300 , 300} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0 ,  0} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ 80  , { 200 , 400} , 0 , 150 } ,   // turn left 1.1s 
	
	{ 17  , { 100 , 100} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 80  , { 200 , 400} , 0 , 150 } ,		// turn left 1.1s 
	
	{ 17  , { 100 , 100} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0 ,  0} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 17  , { 500 , 500} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0   , 0  } , 0 , 0 } ,		// stop
};


/////////////////////////////////////////////////////////////////////////////////
// Menu control
//
/////////////////////////////////////////////////////////////////////////////////
void CarCtrl_Start( void )
{
	g_car_ctrl_state = CarCtrl_IDLE ;
}

void CarCtrl_SuperStart(void)
{
	g_car_ctrl_state = CarCtrl_IDLE ;
	g_CarPlan_Ptr = g_CarPlan_Supper;
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
	if ( g_CarCtrl.car_angle < -45 ) g_CarCtrl.car_angle = -45 ;
	Steer_Moto_Ctrl(STEER_MOTO_POS , g_CarCtrl.car_angle );
}

void CarCtrl_Left( void )
{
	g_CarCtrl.car_angle ++ ;
	if ( g_CarCtrl.car_angle > 45 ) g_CarCtrl.car_angle = 45 ;
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

void CarCtrl_PlanSet( void )
{
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
	else														// execute plan 
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

// �� car_control.c ��

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
		if (scope_send_counter >= 10) // ����10Hz�ķ���Ƶ��
		{
			scope_send_counter = 0;
			
            // ======================================================================
            // == �޸Ĳ��֣������ĸ�ͨ��������
            // ======================================================================
			Scope_Send4Floats(
                (float)g_CarCtrl.car_speed_set[0],  // ͨ��1: ����Ŀ���ٶ�
                (float)g_speed_encoder[0].speed,    // ͨ��2: ����ʵ���ٶ�
                (float)g_CarCtrl.car_speed_set[1],  // ͨ��3: ����Ŀ���ٶ�
                (float)g_speed_encoder[1].speed     // ͨ��4: ����ʵ���ٶ�
            );
            // ======================================================================
		}
		
		CarCtrl_PlanSet();
	}
}