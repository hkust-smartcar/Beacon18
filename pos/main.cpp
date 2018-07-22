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

#include <libsc/button.h>
#include "debug_console.h"
#include <libbase/k60/flash.h>

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
	//0       1           2            3         4      5           6   7          8          9      10
	forwards, chases, rotations, turnRights, turnLefts, keeps, avoids, approachs, backwards, stops, searchs
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
const uint8_t Y_CENTER_OFFEST = 12;
const int16_t CARX = 189/2;
const int16_t CARY = 120/2 + Y_CENTER_OFFEST;
const int16_t O_X_LEFT = 60;
const int16_t O_X_RIGHT = 150;
const int16_t O_NX_OFFSET = 5;
const int16_t O_NX_LEFT = (CARX - (O_X_RIGHT - CARX)) - O_NX_OFFSET;
const float O_X_LEFT_INC_RATIO = 1.0*(((O_X_RIGHT - CARX)+O_NX_OFFSET)-(CARX- O_X_LEFT))/(CARX- O_X_LEFT);

//approach
const int16_t A_X_LEFT = 90;
const int16_t A_X_RIGHT = 110;
const int16_t A_X_CENTER = 99;
const int16_t A_Y = 70;


//pid
int32_t L_count = 0;
int32_t R_count = 0;
const uint16_t crash_cycle=15;
bool atom = false;

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


	Motor_PID L_pid_(L_kp, L_ki, L_kd, 1000, -1000);
	L_pid = &L_pid_;
	L_pid->errorSumBound = 100000;
	Motor_PID R_pid_(R_kp, R_ki, R_kd, 1000, -1000);
	R_pid = &R_pid_;
	R_pid->errorSumBound = 100000;


	PID Dir_pid_(Dir_kp, Dir_ki, Dir_kd, 500, -500);
	Dir_pid = &Dir_pid_;
	Dir_pid->errorSumBound = 10000;
	PID avoid_pid_(avoid_kp, avoid_ki, avoid_kd, 500, -500);
	avoid_pid = &avoid_pid_;
	avoid_pid->errorSumBound = 10000;

	Joystick::Config j_config;
	j_config.id = 0;
	j_config.is_active_low = true;
	Joystick joyStick(j_config);



	DebugConsole menu(&joyStick, lcd, writer);
	menu.PushItem("chSP", &chasing_speed, 10);
	menu.PushItem("fdSP", &finding_speed, 10);
	menu.PushItem("roSP", &rotate_speed, 10);

//    Flash::Config flash_config;
//    Flash flash0(flash_config);
//	menu.SetFlash(&flash0);
//
	auto bFunction = [&menu,&aaction,&seen,&changeSpeedTime,&encoder1,&encoder2](const uint8_t id){

		if(menu.GetFlag()==false && run==false)
		{
			run = true;
			changeSpeedTime = System::Time();
			display_bMeter();
			lcd->SetRegion(Lcd::Rect(90, 0, 128, 160));
			writer->WriteString("star");
			encoder1->Update();
			encoder2->Update();
			R_pid->reset();
			L_pid->reset();
			comm->SendStrLiteral("s");
		}
		else if(menu.GetFlag()==false && run==true)
		{
			run = false;
			aaction = stops;
			actionTarget(aaction);
			//update seen
			if(seen)
			{
				seen = false;
				max_area = 0;
			}
			comm->SendStrLiteral("S");
			//menu.EnterDebug("leave");
		}
		else if(menu.GetFlag()==true && run==true)
		{
			run = false;
			aaction = stops;
			actionTarget(aaction);
			//update seen
			if(seen)
			{
				seen = false;
				max_area = 0;
			}
			comm->SendStrLiteral("S");
		}
//		else if(menu.GetFlag()==true && run==false)
//		{
//
//		}
	};
	Button::Config ButtonConfig;
	ButtonConfig.id = 0;
	ButtonConfig.is_active_low = true;
	ButtonConfig.is_use_pull_resistor = false;
	ButtonConfig.listener_trigger = Button::Config::Trigger::kUp;
	ButtonConfig.listener = bFunction;
	Button button(ButtonConfig);
////////////////////////////////////////////

//var////////////////////////////////////////


	//time
    uint32_t process_time = 0;
    //uint32_t avoid_past_time = 0;
    uint32_t pid_time = 0;
    uint32_t not_find_time = 0;
    uint32_t finding_time = 0;

    bool irSeen = false;
////////////////////////////////////////////

//set//////////////////////////////////////

    run = false;
    chasing_speed = 370;
    finding_speed = 300;
    rotate_speed = 100;

	encoder1->Update();
	encoder2->Update();

	display_bMeter();
	changeSpeedTime = System::Time();

	//menu.EnterDebug("leave");
///////////////////////////////////////////////

	while (true){
		if (tick != System::Time()) {
			tick = System::Time();

//pid////////////////////////////
			if (tick - pid_time >= 10 && run==true) {
				uint32_t time_diff = tick - pid_time;

				encoder1->Update();
				encoder2->Update();
				L_count = encoder1->GetCount() * 50 / (int) time_diff;
				R_count = encoder2->GetCount() * 50 / (int) time_diff;

				if(accCount ==true && firstAcc==true)
				{
					firstAcc=false;
				}
				else if(accCount ==true && firstAcc==false)
				{
					aCountL += L_count;
					aCountR += R_count;

					//move dist
					if(accAction == aaction)
					{

						//move forward
						if(accDis>0)
						{
							if(aCountL>=accDis || aCountR<= - (accDis))
							{
								accCount = false;
								if(aaction==chases)
								{
									Dir_pid->reset();
									L_pid->reset();
									R_pid->reset();
								}
								aaction = actionAfterMove;
								actionTarget(actionAfterMove);
								ssend(accAction);
								atom = false;
							}
						}
						//move backward
						else //(accDis<0)
						{
							if(aCountL<=accDis || aCountR>= abs (accDis))
							{
								accCount = false;
								if(aaction==chases)
								{
									Dir_pid->reset();
									L_pid->reset();
									R_pid->reset();
								}
								aaction = actionAfterMove;
								actionTarget(actionAfterMove);
								ssend(accAction);
								atom = false;
							}
						}
					}
					//action is changed, move is outdated
					else
					{
						accCount = false;
						atom = false;
					}
				}

				if(tick-changeSpeedTime>=200 && aaction!=chases && aaction!=rotations && L_pid->getIsAcc()==true)
				{
					L_pid->setIsAcc(false);
					R_pid->setIsAcc(false);
				}
				SetPower(L_pid->output(L_count), 0);
				SetPower(R_pid->output(-R_count), 1);
				pid_time = System::Time();

//				{
//					char temp[20] = { };
//					sprintf(temp, "al=%d ar=%d", aCountL,aCountR);
//					lcd->SetRegion(Lcd::Rect(0, 20, 128, 160));
//					writer->WriteString(temp);
//					sprintf(temp, "nl=%d", L_pid->getNumError());
//					lcd->SetRegion(Lcd::Rect(0, 40, 128, 160));
//					writer->WriteString(temp);
//					sprintf(temp, "nr=%d", R_pid->getNumError());
//					lcd->SetRegion(Lcd::Rect(0, 60, 128, 160));
//					writer->WriteString(temp);
//				}

			}
			else if(run==false)
			{
				if(aaction != stops)
				{
					if(aaction==chases)
					{
						Dir_pid->reset();
						L_pid->reset();
						R_pid->reset();
					}
					aaction = stops;
					encoder1->Update();
					encoder2->Update();
				}
			}
//pid end/////////////////////////////////

//process///////////////////////////
			if (tick - process_time >= 29 && atom == false && menu.GetFlag()==false) {
			//if(tick - avoid_past_time>=19){
				//avoid_past_time = tick;

				process_time = tick;

////crash///////////////////////////////////////////////////////

//				if(run==true && aaction!= sstate_::backwards && tick - o_target.received_time < 200 && tick-changeSpeedTime>=200 && (
				if(run==true && aaction!= sstate_::backwards && tick-changeSpeedTime>=200 && (
						(!(abs(L_pid->getTarget())<20) && (abs(L_count)<20))
						|| (!(abs(R_pid->getTarget())<20) && (abs(R_count)<20))
					)  && aaction!=rotations && aaction!= searchs)
				{
					if(aaction == sstate_::avoids)
					{
						moveCount(-30, sstate_::backwards, sstate_::forwards); //avoid again if seen obstacle
						aaction = backwards;
					}
//					else if(aaction == sstate_::rotations)
//					{
//						moveCount(40, sstate_::turnLefts, sstate_::rotations);
//						aaction = turnLefts;
//					}
					else if(aaction==chases)
					{
						Dir_pid->reset();
						L_pid->reset();
						R_pid->reset();
						if(Dir_pid->getTarget()>0)
						{
							moveCount(-30, sstate_::backwards, turnLefts);
						}
						else
						{
							moveCount(-30, sstate_::backwards, turnRights);
						}
						aaction = backwards;
					}
					else
					{
						moveCount(-30, sstate_::backwards, aaction);
						aaction = backwards;
					}
					actionTarget(aaction);
					ssend(aaction);

					//update seen
					if(seen)
					{
						seen = false;
						max_area = 0;
					}

					continue;
				}
				else if(run==true && aaction==sstate_::backwards && tick-changeSpeedTime>=200 && (
						(!(abs(L_pid->getTarget())<20) && (abs(L_count)<20))
						|| (!(abs(R_pid->getTarget())<20) && (abs(R_count)<20))
					)  && aaction!=rotations && aaction!= searchs)
				{
					if(abs(L_count)>abs(R_count))
					{
						moveCount(30, sstate_::turnLefts, sstate_::forwards);
						aaction = turnLefts;
					}
					else
					{
						moveCount(30, sstate_::turnRights, sstate_::forwards);
						aaction = turnRights;
					}
					actionTarget(aaction);
					finding_time = 0; //trigger rotations in not seen condition check
					ssend(aaction);

					//update seen
					if(seen)
					{
						seen = false;
						max_area = 0;
					}

					continue;
				}

////crash end/////////////////////////////////

///null turn/////////////////
				else
				if(run==true && tick-changeSpeedTime>=200 && (L_pid->getNumError()>crash_cycle || R_pid->getNumError()>crash_cycle) && aaction!=rotations && aaction!= searchs)
				{
					if(aaction == sstate_::avoids || aaction == sstate_::approachs)
					{
						moveCount(-20, sstate_::backwards, sstate_::forwards); //avoid again if seen obstacle
						aaction = backwards;
					}

					else if(aaction==chases)
					{
						Dir_pid->reset();
						L_pid->reset();
						R_pid->reset();
						if(Dir_pid->getTarget()>0)
						{
							moveCount(-30, sstate_::backwards, turnLefts);
						}
						else
						{
							moveCount(-30, sstate_::backwards, turnRights);
						}
						aaction = backwards;
					}

					else if(aaction!= sstate_::backwards && ((L_pid->getNumError()>crash_cycle && L_pid->getTarget()>0)|| (R_pid->getNumError()>crash_cycle && R_pid->getTarget()<0)))
					{

						moveCount(-30, sstate_::backwards, aaction);
						aaction = backwards;
						actionTarget(aaction);
						ssend(aaction);
					}
					else if(aaction!= sstate_::forwards && ((L_pid->getNumError()>crash_cycle && L_pid->getTarget()<0)|| (R_pid->getNumError()>crash_cycle && R_pid->getTarget()>0)))
					{
						moveCount(20, sstate_::forwards, aaction);
						aaction = forwards;
						actionTarget(aaction);
						ssend(aaction);
					}

					//update seen
					if(seen)
					{
						seen = false;
						max_area = 0;
					}

					continue;
				}
				else if ((aaction == sstate_:: rotations||aaction == sstate_:: searchs) && tick-changeSpeedTime>=500 && (
						(!(abs(L_pid->getTarget())<20) && (abs(L_count)<20))
						|| (!(abs(R_pid->getTarget())<20) && (abs(R_count)<20))
					))
				{
//					if(abs(L_count)>abs(R_count))
					{
						moveCount(30, sstate_::backwards, sstate_::turnLefts);
						aaction = backwards;
					}
//					else
//					{
//						moveCount(30, sstate_::backwards, sstate_::turnRights);
//						aaction = backwards;
//					}
					actionTarget(aaction);
				}
				else if(run==true && tick-changeSpeedTime<200)
				{
					L_pid->setIsCount(false);
					R_pid->setIsCount(false);
				}
				else if(run==true && tick-changeSpeedTime>200)
				{
					L_pid->setIsCount(true);
					R_pid->setIsCount(true);
				}
//null turn end//////////////////

//otarget/////////////////////////////
				if (tick - o_target.received_time < 200 && o_target.target->center.first > 40 && o_target.target->center.first < 170 && aaction!=sstate_::rotations && aaction!=searchs && aaction!=backwards && aaction!=chases) {
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
								if(aaction==chases)
								{
									Dir_pid->reset();
									L_pid->reset();
									R_pid->reset();
								}
								aaction = backwards;
								if(x<CARX){
									moveCount(-20, sstate_::backwards, sstate_::turnRights);
								}
								else
								{
									moveCount(-20, sstate_::backwards, sstate_::turnLefts);
								}
								actionTarget(aaction);
								pid(tick, pid_time);
							}

						}
						else
						{
							if(aaction==chases)
							{
								Dir_pid->reset();
								L_pid->reset();
								R_pid->reset();
							}
							setAnglePower(BeaconAvoidAngleCalculate(x, y), tick, pid_time);
							//avoid_past_time = System::Time();
							aaction = avoids;
							changeSpeedTime = System::Time();
						}
						ssend(aaction);
						if(seen)
						{
							seen = false;
							max_area = 0;
						}
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
						moveCount(20, sstate_::forwards, sstate_::forwards);
						aaction = forwards;
						actionTarget(aaction);
						finding_time = 0; //trigger rotations in not seen condition check
						//pid(tick, pid_time);
					}
				}
//otarget end//////////////////////////////

//irtarget2////////////////////////
				process();

				if (ir_target != NULL && tick - ir_target2.received_time < 200)
				{
					uint16_t x = ir_target2.target->center.first;
					uint16_t y = ir_target2.target->center.second;

					//ir cam
					not_find_time = 0;
					finding_time = 0;

					if (!irSeen) {
						irSeen = true;
						Dir_pid->reset();
					}
					if (!seen) {
						seen = true;
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

//					}
//					else
					{
						if(aaction==chases)
						{
							Dir_pid->reset();
							L_pid->reset();
							R_pid->reset();
						}
						setAnglePower(BeaconApproachAngleCalculate(x, y), tick, pid_time);
						aaction = approachs;
						changeSpeedTime = System::Time();
					}
					ssend(aaction);
					continue;
				}
//ir target 2 end/////////////////////////////////

//ir target/////////////////////////
				if (ir_target != NULL) {	//target find
					led1->SetEnable(true);
					not_find_time = 0;
					finding_time = 0;

					last_beacon = ir_target->center;
					if (!irSeen) {
						irSeen = true;
						Dir_pid->reset();
					}
					if (!seen) {
						seen = true;
					}
					if (ir_target->area > max_area)
						max_area = (ir_target->area + max_area) / 2;
					target_x = target_slope * max_area + target_intercept;
					if (target_x > 320)
						target_x = 320;
					if ((aaction == rotations||aaction == searchs)
							&& ir_target->center.first > target_x)
					{
						if(aaction == rotations)
						{
							aaction = searchs;
							actionTarget(aaction);
						}
						//aaction = keeps;
					}
					else
					{
						aaction = chases;
						actionTarget(aaction);
					}
				}
				else if (irSeen) { //target not find but have seen target before
					if (not_find_time == 0) {
						not_find_time = tick;
						//aaction = keeps;
					} else if (tick - not_find_time > 400) { //target lost for more than 400 ms
						led1->SetEnable(0);
						seen = false;
						irSeen = false;
						max_area = 0;

						if(Dir_pid->getTarget()>0)
						{
							if(aaction!=turnLefts)
							{
								moveCount(30, sstate_::turnLefts, turnRights);
								aaction = turnLefts;
							}
						}
						else
						{
							if(aaction!=turnRights)
							{
								moveCount(30, sstate_::turnRights, turnLefts);
								aaction = turnRights;
							}
						}


//						if(aaction!=backwards)
//						{
//							if(aaction==chases)
//							{
//								Dir_pid->reset();
//								L_pid->reset();
//								R_pid->reset();
//							}
//							moveCount(-20, sstate_::backwards, sstate_::rotations);
//							aaction = backwards;
//							actionTarget(aaction);
//							//pid(tick, pid_time);
//						}

					}
				}
				else if (accCount==true) { //forward move count
					finding_time = tick;
				}
				else { //target not find and have not seen target before
					led1->SetEnable(0);
					if (finding_time == 0 && aaction!=sstate_::forwards)
					{
						if(aaction==chases)
						{
							Dir_pid->reset();
							L_pid->reset();
							R_pid->reset();
						}
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
					else if(finding_time != 0 && aaction == sstate_::rotations && tick-finding_time>3040)//rotate 2000ms
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

//menu///////////////////////////////////
			if(run==false && tick - pid_time>200 && menu.GetFlag()==false)
			{
				char temp[20];
				lcd->SetRegion(Lcd::Rect(90, 20, 128, 160));
				sprintf(temp, "ch%d", chasing_speed);
				writer->WriteString(temp);

				pid_time = tick;
				lcd->SetRegion(Lcd::Rect(90, 0, 128, 160));
				switch(joyStick.GetState())
				{
				case Joystick::State::kUp:
				{
					writer->WriteString("uppp");
					menu.EnterDebug("leave");
					break;
				}
				case Joystick::State::kDown:
				{
					writer->WriteString("down");
					break;
				}
				case Joystick::State::kLeft:
				{
					writer->WriteString("left");
					break;
				}
				case Joystick::State::kRight:
				{
					writer->WriteString("righ");
					break;
				}
				case Joystick::State::kSelect:
				{
					writer->WriteString("sele");
					break;
				}
				case Joystick::State::kIdle:
				{
					writer->WriteString("idle");
					break;
				}
				default:
				{
					writer->WriteString("oooo");
					break;
				}
				}
			}
//menu end///////////////////////////////////
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
		R_pid->settarget(finding_speed);
		if(radAngle<=-1||radAngle==0)
		{
			L_pid->settarget(0);
			//L_pid->reset();
			SetPower(1,0);
		}
		else
		{
			L_pid->settarget(finding_speed* abs(radAngle));
		}


	}
	else //radAngle>0
	{
		L_pid->settarget(finding_speed);
		if(radAngle>=1||radAngle==0)
		{
			R_pid->settarget(0);
			//R_pid->reset();
			SetPower(1,1);
		}
		else
		{
			R_pid->settarget(finding_speed*radAngle);
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

	if(accCount ==true && firstAcc==true)
	{
		firstAcc=false;
	}
	else if(accCount ==true && firstAcc==false)
	{
		aCountL += L_count;
		aCountR += R_count;

		//move dist
		if(accAction == aaction)
		{

			//move forward
			if(accDis>0)
			{
				if(aCountL>=accDis || aCountR<= - (accDis))
				{
					accCount = false;
					if(aaction==chases)
					{
						Dir_pid->reset();
						L_pid->reset();
						R_pid->reset();
					}
					aaction = actionAfterMove;
					actionTarget(actionAfterMove);
					ssend(accAction);
					atom = false;
				}
			}
			//move backward
			else //(accDis<0)
			{
				if(aCountL<=accDis || aCountR>= abs (accDis))
				{
					accCount = false;
					if(aaction==chases)
					{
						Dir_pid->reset();
						L_pid->reset();
						R_pid->reset();
					}
					aaction = actionAfterMove;
					actionTarget(actionAfterMove);
					ssend(accAction);
					atom = false;
				}
			}
		}
		//action is changed, move is outdated
		else
		{
			accCount = false;
			atom = false;
		}
	}

	if(tick-changeSpeedTime>=200 && aaction!=chases && aaction!= rotations && L_pid->getIsAcc()==true)
	{
		L_pid->setIsAcc(false);
		R_pid->setIsAcc(false);
	}
	SetPower(L_pid->output(L_count), 0);
	SetPower(R_pid->output(-R_count), 1);
	pid_time = System::Time();

//				{
//					char temp[20] = { };
//					sprintf(temp, "al=%d ar=%d", aCountL,aCountR);
//					lcd->SetRegion(Lcd::Rect(0, 20, 128, 160));
//					writer->WriteString(temp);
//					sprintf(temp, "nl=%d", L_pid->getNumError());
//					lcd->SetRegion(Lcd::Rect(0, 40, 128, 160));
//					writer->WriteString(temp);
//					sprintf(temp, "nr=%d", R_pid->getNumError());
//					lcd->SetRegion(Lcd::Rect(0, 60, 128, 160));
//					writer->WriteString(temp);
//				}
}

void actionTarget(const sstate_& taction)
{
	if(accCount == true && taction!=accAction)
	{
		accCount = false;
	}
	if(taction!=accAction)
	{
		L_pid->setIsCount(false);
		R_pid->setIsCount(false);
	}
	if(L_pid->getIsAcc()==false && taction!=accAction && (aaction == chases||aaction==rotations))
	{
		L_pid->setIsAcc(true);
		R_pid->setIsAcc(true);

	}
	if(L_pid->getIsAcc()==true && taction!=accAction && (aaction != chases||aaction!=rotations))
	{
		L_pid->setIsAcc(false);
		R_pid->setIsAcc(false);
	}

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
		case searchs:
		{
			L_pid->settarget(80);
			R_pid->settarget(-80);
			changeSpeedTime = System::Time();
			break;
		}
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
		case stops:
		{
			L_pid->settarget(0);
			R_pid->settarget(0);
			L_pid->reset();
			R_pid->reset();
			SetPower(0,0);
			SetPower(0,1);
			changeSpeedTime = System::Time();
			run = false;
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
