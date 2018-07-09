/*
 * main.cpp
 *
 * Author: Wongky
 * Copyright (c) 2018-2019 HKUST SmartCar Team
 * Refer to LICENSE for details
 */


#include <cassert>
#include <cstring>
#include <stdlib.h>
#include <string>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include "libsc/joystick.h"
#include "libsc/battery_meter.h"
#include "libbase/misc_utils_c.h"
#include "image_processing.h"
#include "libutil/misc.h"
#include "config.h"
#include "var.h"

#include <stdlib.h>
#include <cmath>




namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 100000;
			return config;
		}

	}
}

using libsc::System;
using namespace libsc;
using namespace libbase::k60;

//pos
//pos
const uint8_t COUNT_PER_CM = 23;
const float CM_PER_COUNT = 1.0/COUNT_PER_CM;  //0.041666666
const float PI = 22/7.0;
const float DEG_PER_RAD = 180.0/PI; //1 rad = 180/pi
const uint8_t WHEEL_DIST = 12;
const uint8_t Y_CENTER_OFFEST = 12;
const int16_t CARX = 189/2;
const int16_t CARY = 120/2 + Y_CENTER_OFFEST;
const int16_t CARW = 16; //cm
const int16_t XCOOR_IMAGE_OFFSET = 189/2; //lowest middle of image is (0,0)
const int16_t O_X_LEFT = 60;
const int16_t O_X_RIGHT = 150;
const int16_t O_NX_OFFSET = 5;
const int16_t O_NX_LEFT = (CARX - (O_X_RIGHT - CARX)) - O_NX_OFFSET;
const float O_X_LEFT_INC_RATIO = 1.0*(((O_X_RIGHT - CARX)+O_NX_OFFSET)-(CARX- O_X_LEFT))/(CARX- O_X_LEFT);
const float X_CM_PER_PIX = 0.1; //1pixel = 1mm, x-axis
const float Y_CM_PER_PIX = 2;
const int8_t YX_ratio = 20; //20mm/1mm

//pid
int32_t L_count = 0;
int32_t R_count = 0;
int32_t aCountL = 0;
int32_t aCountR = 0;

bool printLCD = false;

inline float BeaconAvoidAngleCalculate(const uint16_t& bx, const uint16_t& by);
inline void setAnglePower(const float& radAngle, const uint32_t& tick, uint32_t& pid_time);
inline void pid(const uint32_t& tick, uint32_t& pid_time);

int main(void)
{
	System::Init();

//init/////////////////////////////////////
	Led Led0(init_led(0));
	led0 = &Led0;
	Led Led1(init_led(1));
	led1 = &Led1;
	BatteryMeter bMeter_(init_bMeter());
	bMeter = &bMeter_;
	St7735r lcd_(init_lcd());
	lcd = &lcd_;
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd->Clear(Lcd::kWhite);
	LcdTypewriter writer_(init_writer());
	writer = &writer_;
	AlternateMotor motor0(init_motor(0));
	L_motor = &motor0;
	AlternateMotor motor1(init_motor(1));
	R_motor = &motor1;
	DirEncoder encoder1_(init_encoder(1));
	encoder1 = &encoder1_;
	DirEncoder encoder2_(init_encoder(0));
	encoder2 = &encoder2_;
	k60::Ov7725 cam_(init_cam());
	cam = &cam_;
	cam->Start();
	Joystick::Config j_config;
	j_config.id = 0;
	Joystick joyStick(j_config);
	uint8_t state = 100;
	JyMcuBt106 bt_(init_bt());
	bt = &bt_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;

	PID L_pid_(L_kp, L_ki, L_kd, 1000, -1000);
	L_pid = &L_pid_;
	L_pid->errorSumBound = 100000;
	PID R_pid_(R_kp, R_ki, R_kd, 1000, -1000);
	R_pid = &R_pid_;
	R_pid->errorSumBound = 100000;

	PID Dir_pid_(Dir_kp, Dir_ki, Dir_kd, 500, -500);
	Dir_pid = &Dir_pid_;
	Dir_pid->errorSumBound = 10000;
	PID avoid_pid_(avoid_kp, avoid_ki, avoid_kd, 500, -500);
	avoid_pid = &avoid_pid_;
	avoid_pid->errorSumBound = 10000;
////////////////////////////////////////////

//var////////////////////////////////////////





	//time
    uint32_t process_time = 0;
    uint32_t past_time = System::Time();
    uint32_t pid_time = 0;

/////////////////////////////////////////

    run = false;
    chasing_speed = 200;
	L_pid->settarget(chasing_speed);
	R_pid->settarget(chasing_speed);

	display_bMeter();


	while (true){
		if (tick != System::Time()) {
			tick = System::Time();
//pid////////////////////////////
			if (run&& tick - pid_time >= 10) {
				uint32_t time_diff = tick - pid_time;
				encoder1->Update();
				encoder2->Update();
				L_count = encoder1->GetCount() * 50 / (int) time_diff;
				R_count = encoder2->GetCount() * 50 / (int) time_diff;
				SetPower(L_pid->output(L_count), 0);
				SetPower(R_pid->output(-R_count), 1);
				pid_time = System::Time();

//				{
//					lcd->SetRegion(Lcd::Rect(0, 20, 128, 160));
//					writer->WriteString("start");
//				}

			}
			else if(run==false)
			{
				SetPower(0,0);
				SetPower(0,1);
			}
///////////////////////////////////

//avoid///////////////////////////
			if (tick - process_time >= 19) {

				process_time = tick;

				if (tick - o_target.received_time < 100) {
					uint16_t x = o_target.target->center.first;
					uint16_t y = o_target.target->center.second;
					if(printLCD)
					{
						char temp[20] = { };
						sprintf(temp, "x=%d y=%d", x,y);
						lcd->SetRegion(Lcd::Rect(0, 40, 128, 160));
						writer->WriteString(temp);
					}
					if ( x > O_X_LEFT && x < O_X_RIGHT)
					{

						if (y> 70){}
						else
						{
							setAnglePower(BeaconAvoidAngleCalculate(x, y), tick, pid_time);
							 past_time = System::Time();
						}
						//								FSM();
						//								continue;
					}
					else if(past_time-tick>=30)
					{
						L_pid->settarget(chasing_speed);
						R_pid->settarget(chasing_speed);
					}
				}
			}
/////////////////////////////////////
//			if(tick - past_time >=15000){
//				SetPower(0, 0);
//				SetPower(0, 1);
//				run = false;
//			}
		}



	}

	return 0;
}

float BeaconAvoidAngleCalculate(const uint16_t& bx, const uint16_t& by)
{
	if(bx<=O_X_LEFT||bx>=O_X_RIGHT)return 0.0;
	//assume beacon position need to avoid

	int16_t dx = bx - CARX;
	int16_t dy = -(by - CARY); //lower pixel is larger
	uint32_t hyp = (dx *dx + dy *dy) ;

	float ratio = 0.0; //speed ratio

	//have dist
	if (hyp != 0)
	{

		//st line
		if (dx == 0)
		{
			//front
			if (dy > 0)
			{
				//turn left
				int16_t ndx=O_X_RIGHT-CARX;
				//ratio = - (1-atan(ndx*1.0/dy));
				ratio = atan(ndx*1.0/dy);
				if(ratio>1||ratio<-1)ratio = 0;
				ratio = - (1-ratio);
			}
			//back
//			else //dy<=0
//			{
//				targetAngle = 0.0;
//				Radius = 0.0;
//			}
		}
		else  //dx != 0
		{
			if (dx > 0)
			{
				//beacon at front right
				if (dy > 0)
				{
					//turn left
					int16_t ndx=O_X_RIGHT-bx;
					//ratio = - (1-atan(ndx*1.0/dy));
					ratio = atan(ndx*1.0/dy);
					if(ratio>1||ratio<-1)ratio = 0;
					ratio = -(1-ratio);
				}
				//beacon at back right
//				else //dy<0
//				{
//					targetAngle = 0.0;
//					Radius = 0.0;
//				}
			}
			else if(dx<0)
			{
				//beacon at front left
				if (dy > 0)
				{
					//turn right
					int16_t nbx = (((bx - O_X_LEFT) * (1+ O_X_LEFT_INC_RATIO))+ O_NX_LEFT)-O_NX_OFFSET;
					int16_t ndx= (nbx - O_NX_LEFT)+O_NX_OFFSET;
					//ratio = 1- atan(ndx*1.0/dy);
					ratio = atan(ndx*1.0/dy);
					if(ratio>1||ratio<-1)ratio = 0;
					ratio = 1-ratio;
				}
				//beacon at back left
//				else //dy<0
//				{
//					targetAngle = 0.0;
//					Radius = 0.0;
//				}
			}
//			else if (dy == 0)
//			{
//				//beacon at right
//				if (dx > 0)
//				{
//					//turn left
//					//targetAngle = - PI/2;
//					//Radius = 0.0;
//					//backward
//				}
//				//beacon at left
//				else  //dx < 0
//				{
//					//turn right
//					//targetAngle = PI/2;
//					//Radius = 0.0;
//					//backward
//				}
//			}
		}
	}

	if(printLCD)
	{
		char temp[20] = { };
		sprintf(temp, "r=%.3f", ratio);
		lcd->SetRegion(Lcd::Rect(0, 60, 128, 160));
		writer->WriteString(temp);
//		sprintf(temp, "dx=%d, dy=%d",dx,dy);
//		lcd->SetRegion(Lcd::Rect(0, 80, 128, 160));
//		writer->WriteString(temp);
	}

	return ratio;

}

void setAnglePower(const float& radAngle, const uint32_t& tick, uint32_t& pid_time)
{
	int tempR = R_pid->getTarget();
	int tempL = L_pid->getTarget();
	//	-ve turn left; +ve turn right
	if(radAngle<0)
	{
		R_pid->settarget(chasing_speed);
		if(radAngle<=-1||radAngle==0)
		{
			L_pid->settarget(0);
			SetPower(0,0);
		}
		else
		{
			L_pid->settarget(chasing_speed* abs(radAngle));
		}


	}
	else //radAngle>0
	{
		L_pid->settarget(chasing_speed);
		if(radAngle>=1||radAngle==0)
		{
			R_pid->settarget(0);
			SetPower(0,0);
		}
		else
		{
			R_pid->settarget(chasing_speed*radAngle);
		}
	}
	if(run&&(tempR != R_pid->getTarget() || tempL!= L_pid->getTarget()))
	{
		pid(tick, pid_time);
	}
	//if(printLCD)
	{
	char temp[20] = { };
	sprintf(temp, "r=%d\tl=%d", R_pid->getTarget(),L_pid->getTarget());
	lcd->SetRegion(Lcd::Rect(0, 80, 128, 160));
	writer->WriteString(temp);
	}
}

void pid(const uint32_t& tick, uint32_t& pid_time)
{
	uint32_t time_diff = tick - pid_time;
	encoder1->Update();
	encoder2->Update();
	L_count = encoder1->GetCount() * 50 / (int) time_diff;
	R_count = encoder2->GetCount() * 50 / (int) time_diff;
	SetPower(L_pid->output(L_count), 0);
	SetPower(R_pid->output(-R_count), 1);
	pid_time = System::Time();
}
