/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */
/*
 * main.cpp
 *
 * Author:Sheldon
 * Copyright (c) 2017-2018 HKUST SmartCar Team
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
#include "camerafilter.h"

namespace libbase {
namespace k60 {

Mcg::Config Mcg::GetMcgConfig() {
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}
}
}

using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;

/////////////////PID//////////////////////
//float L_kp = 2.5;
//float L_ki = 0.02;
//float L_kd = 0;
//float R_kp = 2.5;
//float R_ki = 0.02;
//float R_kd = 0;
//float Dir_kp = 0.;
//float Dir_ki = 1.75;
//float Dir_kd = 0.05;
////////////////algo parm///////////////////
//const uint16_t near_area = 4000;
//const float target_slope = 0.009855697800993502;
//const float target_intercept = 172.55532972120778;
//////////////speed//////////////////////
//const int chasing_speed = 70;
//const int finding_speed = 70;
//const int rotate_speed = 120;
//const int L_out_speed = 120;
//const int R_out_speed = 70;

//enum rotate_state {
//	no, prepare, performing
//};
//enum state_ {
//	forwardstart, search, approach ,chase,keep,avoid ,arrived//};
/* state			info									action
 *
 * forwardstart		after finish one beacon					move forward a short distance
 *					after search(don't found any beacon)
 *
 * search			after forwardstart						rotation to search a beacon
 *
 * approach			after search find a beacon by ir(long	go a roughtly direction
 * 					distance )
 *
 * chase			after sarch find a beacon by grey cam	to approach the beacon with hight precision
 *					(short distance)
 *
 * keep				as twinkle beacon so we some time can	check haveseen and change action to prestate
 * 					see but some time cannot
 *
 * avoid			see a beacon in front of it ,it will	change action to avoid ,but keep prestate as
 * 					turn left or right whenver state is
 *
 * arrived			arrived a beacon(check whether 		 	change the action to forwardstart to ready se
 * 					ditance_y is equal to center.y			arch next beacon
 *
 *
 *
 */
enum avoid_state {
	forw,turnleft,turnright,fastturnleft,fastturnright,stop
};
avoid_state avoid_st=forw;
void send(state_ action, uint8_t &state) {
	if (action == keep)
		return;
	if (state != action) {
		state = action;
		char data[10];
		sprintf(data, "S:%d\n", state);
		bt->SendStr(data);
	}
}

int main() {
	System::Init();
	Led Led0(init_led(0));
	led0 = &Led0;
	Led Led1(init_led(1));
	led1 = &Led1;
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
	BatteryMeter bMeter_(init_bMeter());
	bMeter= &bMeter_;
	//////////////////PID init////////////////////
	IncrementalPidController<int, int> L_pid_(0, L_kp, L_ki, L_kd);
	L_pid = &L_pid_;
	L_pid->SetILimit(0);
	L_pid->SetOutputBound(-1000, 1000);
	IncrementalPidController<int, int> R_pid_(0, R_kp, R_ki, R_kd);
	R_pid = &R_pid_;
	R_pid->SetILimit(0);
	R_pid->SetOutputBound(-1000, 1000);
	PID Dir_pid_(Dir_kp, Dir_ki, Dir_kp);
	Dir_pid = &Dir_pid_;
	Dir_pid->errorSumBound = 10000;
//	Pit::Config pitConfig;
//	pitConfig.channel = 0;
//	pitConfig.count = 75000 * 10;
//	pitConfig.isr = &pid_cycle;
//	Pit pit_(pitConfig);
////	Pit pit_(init_pid_loop());
//	pit = &pit_;
	////////////////Variable init/////////////////
	uint32_t tick = System::Time();
	uint32_t not_find_time = 0;
	bool seen = false;
	int finding_time = 0;
	rotate_state rotate = no;
//	state_ action = forward;
	uint16_t target_x = target_intercept;
	uint32_t max_area = 0;

	Beacon** onptr= nullptr;
	Beacon** notonptr = nullptr;
	const int value_forward_left = 40;
	const int value_forward_right = 170;
	const int value_turn_left_right = 100;//if x value is smaller than the range of it,it will turn left /////////need tune
	const int value_need_fast_turn=50;//if y value is larger than the range of it,it will need fast turn/////need tune
	state_ prestate = arrived;
	int  haveseen = 0;// 0 =not seen beacon 1 = seen beacon by ir cam 2= seen beacon by grey cam
	int arrived_y = 90;
	int search_time = 0 ;
	int forward_time =0;
	/////////////////For Dubug////////////////////
	uint8_t state = 100;
	JyMcuBt106 bt_(init_bt());
	bt = &bt_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;

//	lcd->SetRegion(Lcd::Rect(0,0,160,15));
//	char data[20] ={};
//	sprintf(data,"%f",bMeter->GetVoltage());
//	writer->WriteBuffer(data,20);

	uint32_t pkgRecTime = 0;
	uint32_t pid_time = System::Time();
	////////////////Main loop////////////////////////
	while (1) {
		//if (tick != System::Time() && run) {
		if (tick != System::Time()) {
			tick = System::Time();
			if (tick % 10 == 0) {
							uint32_t time_diff = System::Time() - pid_time;
							encoder1->Update();
							encoder2->Update();
							int32_t reading1 = encoder1->GetCount() * 10 / time_diff;
							int32_t reading2 = encoder2->GetCount() * 10 / time_diff;
							SetPower(GetMotorPower(0) + L_pid->Calc(reading1), 0);
							SetPower(GetMotorPower(1) + R_pid->Calc(-reading2), 1);
							pid_time = System::Time();
						}
			if (tick % 20 == 0) {
				///////////////decision making///////////////
				buf = cam->LockBuffer();
				////////////init value///////////////////////
				ir_target = NULL;
				///////////////process image/////////////////
				process(seen);
				if (tick - ir_target2.received_time < 100) {
					onptr = &ir_target2.target;

				} else {
					//onptr=nullptr;
				}
				//if (tick - o_target.received_time < 1000) {
				if(pkgRecTime!=o_target.received_time){
					notonptr = &o_target.target;
					pkgRecTime = o_target.received_time;

				} else {

					//notonptr=nullptr;
				}


				if(notonptr!=nullptr){//find obstacle no need to update prestate as we need prestate after avoid the obstacle
					action =avoid;
				}else
				if(ir_target!=nullptr||onptr!=nullptr){//find beacon
					search_time=0;
					if(onptr==nullptr){//find on-beacon in long distance=>ir!=null
						action =approach;
						prestate = approach;
						haveseen = 1;

					}else if(onptr!=nullptr){//find a on-beacon in short distance=>ir!=null

						action =chase;
						prestate = chase;
						haveseen =2 ;
						if((*onptr)->center.second==arrived_y){
							action =arrived;
							prestate = arrived;

						}
					}

				}else
				if((prestate==approach||prestate==chase)&&(ir_target!=nullptr||onptr!=nullptr)){
					action =keep;


				}else
				if(prestate==arrived){//just start or finished one beacon and not found next
					action=forwardstart;
					prestate=forwardstart;
					haveseen=0;

				}else
				if(prestate==forwardstart&&(System::Time()-forward_time)>=1000){
					action=search;
					prestate=search;
					haveseen=0;
					search_time = tick ;
					forward_time=0;

				}else
				if(prestate==search&&haveseen==0&&(System::Time()-search_time)>=1000){//search but not seen anything
					action=forwardstart;
					prestate=forwardstart;
					search_time=0;
				}



				switch (action) {
				case forwardstart:
					L_pid->SetSetpoint(finding_speed);
					R_pid->SetSetpoint(finding_speed);
					break;
				case search:
					L_pid->SetSetpoint(rotate_speed);
					R_pid->SetSetpoint(-rotate_speed);
					break;
				case approach:
					int diff;
					diff = Dir_pid->output(target_x, ir_target->center.first);
					L_pid->SetSetpoint(chasing_speed - diff);
					R_pid->SetSetpoint(chasing_speed + diff);
					break;
				case chase:
					L_pid->SetSetpoint(L_out_speed);
					R_pid->SetSetpoint(R_out_speed);
					break;
				case keep:
					break;
				case avoid:

					int speed =(L_pid->GetSetpoint()+R_pid->GetSetpoint())/2;
					int difff = Dir_pid->output(findtargetx((*notonptr)->center.first,(*notonptr)->center.second,105,speed), ir_target->center.first);
					switch(avoid_st){
					case forw:
						L_pid->SetSetpoint(chasing_speed - difff);
						R_pid->SetSetpoint(chasing_speed + difff);
						break;
					case turnleft:
						L_pid->SetSetpoint(chasing_speed - difff);
						R_pid->SetSetpoint(chasing_speed + difff);break;
					case turnright:
						L_pid->SetSetpoint(chasing_speed - difff);
						R_pid->SetSetpoint(chasing_speed + difff);break;
//					case fastturnleft:
//						L_pid->SetSetpoint(findtargetx((*notonptr)->center.first,(*notonptr)->center.second,90,L_pid->GetSetpoint()));
//						R_pid->SetSetpoint(findtargetx((*notonptr)->center.first,(*notonptr)->center.second,90,R_pid->GetSetpoint()));
//						break;
//					case fastturnright:
//						L_pid->SetSetpoint(findtargetx((*notonptr)->center.first,(*notonptr)->center.second,90,L_pid->GetSetpoint()));
//						R_pid->SetSetpoint(findtargetx((*notonptr)->center.first,(*notonptr)->center.second,90,R_pid->GetSetpoint()));
//						break;
					case stop:
						L_pid->SetSetpoint(0);
						R_pid->SetSetpoint(0);break;
					}
					break;
				}
				send(action, state);
				cam->UnlockBuffer();
				int time = System::Time();
				char data[20];
//				lcd->SetRegion(libsc::Lcd::Rect(0,15,60,12));
//				switch(action){//forwardstart, search, approach ,chase,keep,avoid ,arrived
//				case forwardstart:
//					writer_.WriteString("forwardstart");
//					break;
//				case search:
//					writer_.WriteString("search");
//					break;
//				case approach:
//					writer_.WriteString("approach");
//					break;
//				case chase :
//					writer_.WriteString("chase");
//					break;
//				case keep:
//					writer_.WriteString("keep");
//					break;
//				case avoid:
//					writer_.WriteString("avoid");
//					break;
//				case arrived:
//					writer_.WriteString("arrived");
//					break;
//				default:
//					writer_.WriteString("n o");
//					break;
//				}
//				lcd->SetRegion(libsc::Lcd::Rect(60,15,60,12));
//				sprintf(data,"T:%d",time-tick);
//				writer_.WriteString(data);
//				lcd->SetRegion(libsc::Lcd::Rect(0,27,60,12));
//				if(ir_target!=nullptr){
//					sprintf(data,"ir:%d",1);
//					writer_.WriteString(data);
//				}else{
//					sprintf(data,"ir:%d",0);
//					writer_.WriteString(data);
//				}
//				lcd->SetRegion(libsc::Lcd::Rect(0,39,60,12));
//				if(onptr!=nullptr){
//					sprintf(data,"on:%d",1);
//					writer_.WriteString(data);
//				}else{
//					sprintf(data,"on:%d",0);
//					writer_.WriteString(data);
//				}
				lcd->SetRegion(libsc::Lcd::Rect(61,39,60,12));
				if(notonptr!=nullptr){
					sprintf(data,"noton:%d",1);
					writer_.WriteString(data);
				}else{
					sprintf(data,"noton:%d",0);
					writer_.WriteString(data);
				}
				lcd->SetRegion(libsc::Lcd::Rect(0,51,160,12));
				sprintf(data,"time:%d",(System::Time()-search_time));
				writer_.WriteString(data);

			}
		}
	}
	return 0;
}
