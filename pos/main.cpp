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


enum sstate_ {
	//0       1           2            3         4      5           6   7          8          9
	forwards, chases, rotations, turnRights, turnLefts, keeps, avoids, approachs, backwards, stops
};

bool printLCD = false;

void ssend (sstate_ state)
{
	BitConsts a;
	Byte out[4];
	bt->SendBuffer(&a.kSTART, 1);
	Byte size[1] = { 4 };
	bt->SendBuffer(size, 1);
	out[0] = state & 0xFF;
	out[1] = ir_target != NULL?1:0;
	out[2] = tick - o_target.received_time < 200? 1:0;
	out[3] = tick - ir_target2.received_time < 200?1:0;
	bt->SendBuffer(out, 4);
	bt->SendBuffer(&a.kEND, 1);

	if(printLCD)
	{
		char temp[20] = { };
		switch(state)
		{
		case 0:
			sprintf(temp, "s=for");
			break;
		case 1:
			sprintf(temp, "s=chase");
			break;
		case 2:
			sprintf(temp, "s=rot");
			break;
		case 3:
			sprintf(temp, "s=turnR");
			break;
		case 4:
			sprintf(temp, "s=turnL");
			break;
		case 5:
			break;
		case 6:
			sprintf(temp, "s=avoid");
			break;
		case 7:
			sprintf(temp, "s=app");
			break;
		case 8:
			sprintf(temp, "s=back");
			break;
		case 9:
			sprintf(temp, "s=stop");
			break;
		default:
			sprintf(temp, "s=");
		}
		lcd->SetRegion(Lcd::Rect(100, 110, 128, 160));
		writer->WriteString(temp);
	}
}

//pos
const uint8_t COUNT_PER_CM = 200;
//const float CM_PER_COUNT = 1.0/COUNT_PER_CM;  //0.041666666
//const float PI = 22/7.0;
//const float DEG_PER_RAD = 180.0/PI; //1 rad = 180/pi
//const uint8_t WHEEL_DIST = 12;
const uint8_t Y_CENTER_OFFEST = 12;
const int16_t CARX = 189/2;
const int16_t CARY = 120/2 + Y_CENTER_OFFEST;
//const int16_t CARW = 16; //cm
//const int16_t XCOOR_IMAGE_OFFSET = 189/2; //lowest middle of image is (0,0)
const int16_t O_X_LEFT = 60;
const int16_t O_X_RIGHT = 150;
const int16_t O_NX_OFFSET = 5;
const int16_t O_NX_LEFT = (CARX - (O_X_RIGHT - CARX)) - O_NX_OFFSET;
const float O_X_LEFT_INC_RATIO = 1.0*(((O_X_RIGHT - CARX)+O_NX_OFFSET)-(CARX- O_X_LEFT))/(CARX- O_X_LEFT);
//const float X_CM_PER_PIX = 0.1; //1pixel = 1mm, x-axis
//const float Y_CM_PER_PIX = 2;
//const int8_t YX_ratio = 20; //20mm/1mm

//approach
const int16_t A_X_LEFT = 90;
const int16_t A_X_RIGHT = 110;
const int16_t A_X_CENTER = 99;
const int16_t A_Y = 70;


//pid
int32_t L_count = 0;
int32_t R_count = 0;

//move
int32_t aCountL = 0;
int32_t aCountR = 0;
bool accCount = false;
bool firstAcc = false;
int32_t accDis = 0;
sstate_ accAction = keeps;
sstate_ actionAfterMove = keeps;
sstate_ aaction = keeps;
uint32_t changeSpeedTime = 0;

inline float BeaconAvoidAngleCalculate(const uint16_t& bx, const uint16_t& by);
inline float BeaconApproachAngleCalculate(const uint16_t& bx, const uint16_t& by);
inline void setAnglePower(const float& radAngle, const uint32_t& tick, uint32_t& pid_time);
inline void pid(const uint32_t& tick, uint32_t& pid_time);
inline void actionTarget(const sstate_& taction);
void moveCount(const int& cmDis, const sstate_& nowA, const sstate_& nextA);

//void ssend(sstate_ state) {
//	if (state == keeps)
//		return;
//	else {
//		char data[10];
//		sprintf(data, "S:%d\n", state);
//		bt->SendStr(data);
//	}
//}

int main(void)
{
	System::Init();

//init/////////////////////////////////////
	JyMcuBt106 bt_(init_bt());
	bt = &bt_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;

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
	//uint8_t state = 100;


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
    //uint32_t avoid_past_time = System::Time();
    uint32_t pid_time = 0;
    uint32_t not_find_time = 0;
    uint32_t finding_time = 0;
////////////////////////////////////////////

//set//////////////////////////////////////

    run = false;
    chasing_speed = 200;
    finding_speed = 150;
    rotate_speed = 100+50;
	L_pid->settarget(finding_speed);
	R_pid->settarget(finding_speed);
	encoder1->Update();
	encoder2->Update();

	display_bMeter();
	changeSpeedTime = System::Time();
///////////////////////////////////////////////

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

				if(accCount ==true && firstAcc==true)
				{
					firstAcc=false;
					continue;
				}
				else if(accCount ==true && firstAcc==false)
				{
					aCountL += L_count;
					aCountR += R_count;
				}

				//move dis
				if(accCount == true && firstAcc==false)
				{
//					{
//						char temp[20] = { };
//						sprintf(temp, "al=%d ar=%d", aCountL,aCountR);
//						lcd->SetRegion(Lcd::Rect(0, 20, 128, 160));
//						writer->WriteString(temp);
//						sprintf(temp, "dis=%d", accDis);
//						lcd->SetRegion(Lcd::Rect(0, 40, 128, 160));
//						writer->WriteString(temp);
//					}
					if(accAction == aaction)
					{
						ssend(accAction);
						//move forward
						if(accDis>0)
						{
							if(aCountL>=accDis || aCountR<= - (accDis))
							{
								accCount = false;
								aaction = actionAfterMove;
								actionTarget(actionAfterMove);
							}
						}
						//move backward
						else //(accDis<0)
						{
							if(aCountL<=accDis || aCountR>= abs (accDis))
							{
								accCount = false;
								aaction = actionAfterMove;
								actionTarget(actionAfterMove);
							}
						}
					}
					 //action is changed, move is outdated
					else
					{
						accCount = false;
					}
				}



			}
			else if(run==false)
			{
				SetPower(0,0);
				SetPower(0,1);
				aaction = stops;
			}
//pid end/////////////////////////////////

//process///////////////////////////
			if (tick - process_time >= 29) {

				process_time = tick;
				process();

				//update seen
				if (ir_target == NULL)
				{
					if (seen) { //target not find but have seen target before
						if (not_find_time == 0) {
							not_find_time = tick;
							//aaction = keeps;
						} else if (tick - not_find_time > 400) { //target lost for more than 400 ms
							led1->SetEnable(0);
							seen = false;
							max_area = 0;
						}
					}
				}
				else //have ir
				{
					not_find_time = 0;
					if (!seen) {
						seen = true;
						Dir_pid->reset();
					}
				}

//otarget/////////////////////////////

				//crash
				if(run==true && aaction!= sstate_::backwards && tick - o_target.received_time < 200 && tick-changeSpeedTime>=200 && (
						(!(abs(L_pid->getTarget())<20) && (abs(L_count)<20))
						|| (!(abs(R_pid->getTarget())<20) && (abs(R_count)<20))
					))
				{
					if(aaction == sstate_::avoids)
					{
						moveCount(-30, sstate_::backwards, sstate_::forwards); //avoid again if seen obstacle
					}
					else moveCount(-30, sstate_::backwards, aaction);
					aaction = backwards;
					actionTarget(aaction);
					ssend(aaction);
					continue;
				}
				else if(run==true && aaction==sstate_::backwards && tick-changeSpeedTime>=200 && (
						(!(abs(L_pid->getTarget())<20) && (abs(L_count)<20))
						|| (!(abs(R_pid->getTarget())<20) && (abs(R_count)<20))
					))
				{
					if(abs(L_count)>abs(R_count))
					{
						moveCount(50, sstate_::turnLefts, sstate_::forwards);
						aaction = turnLefts;
					}
					else
					{
						moveCount(50, sstate_::turnRights, sstate_::forwards);
						aaction = turnRights;
					}
					actionTarget(aaction);
					finding_time = 0; //trigger rotations in not seen condition check
					ssend(aaction);
					continue;
				}

//				//forward crash
//				if(run==true && aaction!= sstate_::backwards && tick - o_target.received_time < 200 && tick-changeSpeedTime>=200 && (
//						(!(L_pid->getTarget()<20 && L_pid->getTarget()>=0) && (L_count<20 && L_count>=0))
//						|| (!(R_pid->getTarget()<20 && R_pid->getTarget()>=0) && ((R_count)>-20) && (R_count<=0))
//					))
//				{
//					if(aaction == sstate_::avoids)
//					{
//						moveCount(-30, sstate_::backwards, sstate_::forwards);
//					}
//					else moveCount(-30, sstate_::backwards, aaction);
//					aaction = backwards;
//					actionTarget(aaction);
//					ssend(aaction);
//					continue;
//				}
//
//				//backward crash
//				if(run==true && aaction!= sstate_::forwards && tick-changeSpeedTime>=200 && (
//						(!(L_pid->getTarget()>-20 && L_pid->getTarget()<=0) && (L_count>-20 && L_count<=0))
//						|| (!(R_pid->getTarget()>-20 && R_pid->getTarget()<=0) && ((R_count)<20) && (R_count>=0))
//					))
//				{
//					if(aaction == sstate_::avoids)
//					{
//						moveCount(30, sstate_::forwards, sstate_::forwards);
//					}
//					else moveCount(30, sstate_::forwards, aaction);
//					aaction = forwards;
//					actionTarget(aaction);
//					ssend(aaction);
//					continue;
//				}

				if (tick - o_target.received_time < 200 && o_target.target->center.first > 40 && o_target.target->center.first < 170 && aaction!=sstate_::rotations && !(aaction==backwards)) {
					uint16_t x = o_target.target->center.first;
					uint16_t y = o_target.target->center.second;
					led0->SetEnable(true);
					if(printLCD)
					{
						char temp[20] = { };
						sprintf(temp, "x=%d y=%d", x,y);
						lcd->SetRegion(Lcd::Rect(0, 20, 128, 160));
						writer->WriteString(temp);
					}
					if ( x > O_X_LEFT && x < O_X_RIGHT)
					{

						// too close
						if (y> 60){
							if(aaction!=backwards)
							{

								aaction = backwards;
								if(x<CARX){
									moveCount(-40, sstate_::backwards, sstate_::turnRights);
								}
								else
								{
									moveCount(-40, sstate_::backwards, sstate_::turnLefts);
								}
								actionTarget(aaction);
								pid(tick, pid_time);
							}

						}
						else
						{
							setAnglePower(BeaconAvoidAngleCalculate(x, y), tick, pid_time);
							//avoid_past_time = System::Time();
							aaction = avoids;
							changeSpeedTime = System::Time();
						}
						ssend(aaction);
						continue;
					}
				}
				else if(tick - o_target.received_time >= 200)
				{
					led0->SetEnable(false);

					//aviod for some time
					if(aaction == avoids)
					{
						//forward some distance before rotation
						//moveCount(40, sstate_::forwards, sstate_::rotations);
						moveCount(40, sstate_::forwards, sstate_::forwards);
						aaction = forwards;
						actionTarget(aaction);
						finding_time = 0; //trigger rotations in not seen condition check
						//pid(tick, pid_time);
					}
				}
//otarget end//////////////////////////////


//irtarget2////////////////////////
				if (ir_target != NULL && tick - ir_target2.received_time < 100)
				{
					uint16_t x = ir_target2.target->center.first;
					uint16_t y = ir_target2.target->center.second;

					//ir cam
					not_find_time = 0;
					finding_time = 0;

					if (!seen) {
						seen = true;
						Dir_pid->reset();
					}

					led1->SetEnable(true);
					if(printLCD)
					{
						char temp[20] = { };
						sprintf(temp, "x=%d y=%d", x,y);
						lcd->SetRegion(Lcd::Rect(0, 60, 128, 160));
						writer->WriteString(temp);
					}

					//too close, circle
//					if (y>= 20 && x>=A_X_LEFT && x<=A_X_RIGHT){
//
//					}
//					else
					{
						setAnglePower(BeaconApproachAngleCalculate(x, y), tick, pid_time);
						aaction = approachs;
						changeSpeedTime = System::Time();
					}
					ssend(aaction);
					continue;
				}
//				else
//				{
//					led1->SetEnable(false);
//				}
//ir target 2 end/////////////////////////////////

//ir target/////////////////////////
				if (ir_target != NULL) {	//target find
					led1->SetEnable(true);
					not_find_time = 0;
					finding_time = 0;
					last_beacon = ir_target->center;
					if (!seen) {
						seen = true;
						Dir_pid->reset();
					}
					if (ir_target->area > max_area)
						max_area = (ir_target->area + max_area) / 2;
					target_x = target_slope * max_area + target_intercept;
					if (target_x > 320)
						target_x = 320;
					if (aaction == rotations
							&& ir_target->center.first > target_x)
					{
						//aaction = keeps;
					}
					else
					{
						aaction = chases;
						actionTarget(aaction);
					}
				}
				else if (seen) { //target not find but have seen target before
					if (not_find_time == 0) {
						not_find_time = tick;
						//aaction = keeps;
					} else if (tick - not_find_time > 400) { //target lost for more than 400 ms
						led1->SetEnable(0);
						seen = false;
						max_area = 0;

						moveCount(-40, sstate_::backwards, sstate_::rotations);
						aaction = backwards;
						actionTarget(aaction);
						//pid(tick, pid_time);

					}
				}
				else if (accCount==true) { //forward move count
					finding_time = tick;
				}
				else { //target not find and have not seen target before
					led1->SetEnable(0);
					if (finding_time == 0 && aaction!=sstate_::forwards)
					{
						moveCount(30, sstate_::forwards, sstate_::rotations);
						aaction = forwards;
						actionTarget(aaction);
						finding_time = tick;
					}
					else if(finding_time ==0 && aaction== sstate_::forwards)
					{
						aaction = rotations;
						actionTarget(aaction);
						finding_time = tick;
					}
					else if(finding_time != 0 && aaction == sstate_::rotations && tick-finding_time>2040)//rotate 2000ms
					{
						finding_time = 0;
					}
					else if(finding_time != 0 && aaction != sstate_::rotations) //unknown
					{
						finding_time = 0;
					}

				}
//ir target end//////////////////////////////////


				ssend(aaction);
			}
//process end///////////////////////////////////
		}



	}

	return 0;
}

float BeaconAvoidAngleCalculate(const uint16_t& bx, const uint16_t& by)
{
	if(bx<=O_X_LEFT||bx>=O_X_RIGHT)return 2.0;
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
		lcd->SetRegion(Lcd::Rect(0, 40, 128, 160));
		writer->WriteString(temp);
	}

	return ratio;

}

float BeaconApproachAngleCalculate(const uint16_t& bx, const uint16_t& by)
{
	//if((bx>=A_X_LEFT&&bx<=A_X_RIGHT) || by>=A_Y)return 2.0;
	//assume beacon position need to approach

	int16_t dx = bx - A_X_CENTER;
	int16_t dy = -(by - A_Y); //lower pixel is larger
	uint32_t hyp = (dx *dx + dy *dy) ;

	float ratio = 2.0; //speed ratio

	//have dist
	if (hyp != 0)
	{

		//st line
//		if (dx == 0)
//		{
//			//front
//			if (dy > 0)
//			{
////				targetAngle = 0.0;
////				Radius = 0.0;
//			}
//			//back
////			else //dy<=0
////			{
////				targetAngle = 0.0;
////				Radius = 0.0;
////			}
//		}
		if(dx != 0)
		{
			if (dx > 0)
			{
				//beacon at front right
				if (dy > 0)
				{
					//turn right
					int16_t ndx=bx-A_X_CENTER;
					ratio = atan(ndx*1.0/dy);
					if(ratio>1||ratio<-1)ratio = 0;
					ratio = (1-ratio);
				}
				//beacon at back right
				else if(dy<0)
				{
					//turn right
					int16_t ndx=bx-A_X_CENTER;
					ratio = atan(ndx*1.0/abs(dy));
					if(ratio>1||ratio<-1)ratio = 0;
					ratio = (1-ratio);
				}
			}
			else if(dx<0)
			{
				//beacon at front left
				if (dy > 0)
				{
					//turn left
					int16_t ndx=A_X_CENTER-bx;
					ratio = atan(ndx*1.0/dy);
					if(ratio>1||ratio<-1)ratio = 0;
					ratio = - (1-ratio);
				}
				//beacon at back left
				else if(dy<0)
				{
					//turn left
					int16_t ndx=A_X_CENTER-bx;
					ratio = atan(ndx*1.0/abs(dy));
					if(ratio>1||ratio<-1)ratio = 0;
					ratio = - (1-ratio);
				}
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
		lcd->SetRegion(Lcd::Rect(0, 80, 128, 160));
		writer->WriteString(temp);
	}

	return ratio;

}

void setAnglePower(const float& radAngle, const uint32_t& tick, uint32_t& pid_time)
{
	if(radAngle >=2)
	{
		return;
	}

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
			SetPower(0,1);
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
	if(printLCD)
	{
		char temp[20] = { };
		sprintf(temp, "r=%d\tl=%d", R_pid->getTarget(),L_pid->getTarget());
		lcd->SetRegion(Lcd::Rect(0, 100, 128, 160));
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

	if(accCount ==true && firstAcc==true)
	{
		firstAcc=false;
		return;
	}
	else if(accCount ==true && firstAcc==false)
	{
		aCountL += L_count;
		aCountR += R_count;
	}

	//move dis
	if(accCount == true && firstAcc==false)
	{
//					{
//						char temp[20] = { };
//						sprintf(temp, "al=%d ar=%d", aCountL,aCountR);
//						lcd->SetRegion(Lcd::Rect(0, 20, 128, 160));
//						writer->WriteString(temp);
//						sprintf(temp, "dis=%d", accDis);
//						lcd->SetRegion(Lcd::Rect(0, 40, 128, 160));
//						writer->WriteString(temp);
//					}
		if(accAction == aaction)
		{
			ssend(accAction);
			//move forward
			if(accDis>0)
			{
				if(aCountL>=accDis || aCountR<= - (accDis))
				{
					accCount = false;
					aaction = actionAfterMove;
					actionTarget(actionAfterMove);
				}
			}
			//move backward
			else //(accDis<0)
			{
				if(aCountL<=accDis || aCountR>= abs (accDis))
				{
					accCount = false;
					aaction = actionAfterMove;
					actionTarget(actionAfterMove);
				}
			}
		}
		 //action is changed, move is outdated
		else
		{
			accCount = false;
		}
	}
}

void actionTarget(const sstate_& taction)
{
	switch(taction)
	{
		case forwards:
			L_pid->settarget(finding_speed);
			R_pid->settarget(finding_speed);
			changeSpeedTime = System::Time();
			break;
		case backwards:
			L_pid->settarget(-finding_speed);
			R_pid->settarget(-finding_speed);
			changeSpeedTime = System::Time();
			break;
		case rotations:
			L_pid->settarget(rotate_speed);
			R_pid->settarget(-rotate_speed);
			changeSpeedTime = System::Time();
			break;
		case turnRights:
			L_pid->settarget(finding_speed);
			R_pid->settarget((int) (finding_speed * 0.67));
			changeSpeedTime = System::Time();
			break;
		case turnLefts:
			L_pid->settarget((int) (finding_speed * 0.67));
			R_pid->settarget(finding_speed);
			changeSpeedTime = System::Time();
			break;
		case chases:
		{
			int diff;
			Dir_pid->settarget(target_x);
			diff = Dir_pid->output(ir_target->center.first);
			diff = chasing_speed * diff / 100;
			L_pid->settarget(chasing_speed - diff);
			R_pid->settarget(chasing_speed + diff);
			if (chasing_speed < 150)
				L_pid->settarget(chasing_speed);
			changeSpeedTime = System::Time();
			break;
		}
		default:
			break;
	}
}

void moveCount(const int& cmDis, const sstate_& nowA, const sstate_& nextA)
{
	accDis = (cmDis*COUNT_PER_CM);
	accCount=true;
	firstAcc = true;
	aCountL = 0;
	aCountR = 0;
	accAction = nowA;
	actionAfterMove = nextA;
}
