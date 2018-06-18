
#include <cassert>
#include <cstring>
#include <stdlib.h>
#include <string>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
//#include "libsc/joystick.h"
#include "libsc/battery_meter.h"
//#include "libbase/k60/pit.h"
#include "libbase/misc_utils_c.h"
#include "libutil/incremental_pid_controller.h"
#include "pid.h"
#include "image_processing.h"
#include "motor_util.h"
#include "libutil/misc.h"
#include "config.h"
#include "var.h"

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
float L_kp = 2.5;
float L_ki = 0.02;
float L_kd = 0;
float R_kp = 2.5;
float R_ki = 0.02;
float R_kd = 0;
float Dir_kp = 0.5;
float Dir_ki = 0.1;
float Dir_kd = 0.05;
//////////////algo parm///////////////////
const uint16_t near_area = 4000;
const float target_slope = 0.009855697800993502;
const float target_intercept = 172.55532972120778;

////////////speed//////////////////////
// const int chasing_speed = 100;
// const int finding_speed = 70;
// const int rotate_speed = 120;
// const int L_out_speed = 120;
// const int R_out_speed = 70;

const int chasing_speed = 70;
const int finding_speed = 50;
const int rotate_speed = 70;
const int L_out_speed = 50;
const int R_out_speed = 22;

enum rotate_state {
	no, prepare, performing,turnleft,turnright,fastturnleft,fastturnright,turn180
};
enum state_ {
	forward, chase, rotation, out, keep,avoid
};

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
	//////////////////PID init////////////////////
	IncrementalPidController<int, int> L_pid(0, L_kp, L_ki, L_kd);
	L_pid.SetILimit(0);
	L_pid.SetOutputBound(-600, 600);
	IncrementalPidController<int, int> R_pid(0, R_kp, R_ki, R_kd);
	R_pid.SetILimit(0);
	R_pid.SetOutputBound(-600, 600);
//	IncrementalPidController<int, int> Dir_pid(0, Dir_kp, Dir_ki, Dir_kp);
//	Dir_pid.SetILimit(0);
//	R_pid.SetOutputBound(-200, 200);
	PID Dir_pid(Dir_kp, Dir_ki, Dir_kp);
	Dir_pid.errorSumBound = 10000;

	////////////////Variable init/////////////////
	uint32_t tick = System::Time();
	uint32_t not_find_time = 0;
	bool seen = false;
	int finding_time = 0;
	rotate_state rotate = no;
	state_ action = forward;
	int presbeadir = 0 ; //0:no need ,1:left ,2:right
	uint16_t target_x = target_intercept;
	uint32_t max_area = 0;
	Beacon** onptr = NULL;////////beacon is on
	Beacon** notonptr = NULL;///////beacon is not on
	const int value_forward_left = 30;
	const int value_forward_right = 140;
	const int value_turn_left_right = 100;//if x value is smaller than the range of it,it will turn left /////////need tune
	const int value_need_fast_turn=40;//if y value is larger than the range of it,it will need fast turn/////need tune
	/////////////////For Dubug////////////////////
	uint8_t state = 100;
	bool run = false;
	bool restart = false;
	JyMcuBt106 bt_(init_bt(run));
	bt = &bt_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;

//	////////////////Main loop////////////////////////
	while (1) {


		if (tick != System::Time() ) {
			tick = System::Time();

			////////////////////PID///////////////////////
			if (tick % 10 == 0) {
				encoder1->Update();
				encoder2->Update();
				SetPower(GetMotorPower(0) + L_pid.Calc(encoder1->GetCount()), 0);
				SetPower(GetMotorPower(1) + R_pid.Calc(-encoder2->GetCount()), 1);
			}
			////////////////////////////////////////new

			if (tick - ir_target2.received_time < 100) {
								onptr = &ir_target2.target;

							} else {
								onptr=nullptr;
			}
			if (tick - o_target.received_time < 100) {
								notonptr = &o_target.target;

							} else {

								notonptr=nullptr;
			}
			////////////////////////////////////////////////
			if (tick % 15 == 0) {
				char data[20];

				///////////////decision making///////////////
				buf = cam->LockBuffer();
				////////////init value///////////////////////
				ir_target = NULL;
				///////////////process image/////////////////
				process(seen);
				////////////////////////////////////////////find a obstacle and u need to avoid
				if (notonptr !=nullptr&&(*notonptr)->center.first>value_forward_left&&(*notonptr)->center.first<value_forward_right){

					action =  avoid;
					if((*notonptr)->center.first>value_turn_left_right &&(*notonptr)->center.second>value_need_fast_turn){
						rotate=fastturnleft;
						if(seen){
							presbeadir=2;
						}
					}
					if((*notonptr)->center.first<=value_turn_left_right&&(*notonptr)->center.second>value_need_fast_turn){
						rotate=fastturnright;
						if(seen){
							presbeadir=1;
						}
					}
					if((*notonptr)->center.first>value_turn_left_right&&(*notonptr)->center.second<=value_need_fast_turn){

						rotate=turnleft;
					}
					if((*notonptr)->center.first<=value_turn_left_right&&(*notonptr)->center.second<=value_need_fast_turn){
						rotate=turnright;
					}

				}else
				/////////////////////////////////////////
				if (ir_target != NULL&&onptr !=nullptr){
					presbeadir=0;
					const int close_y=30;//when y is larger than it no need to turn ,but need to prepare to rotate /////////need tune
					const int grey_midline =80; /////////need tune
					lcd_.SetRegion(libsc::Lcd::Rect(0,60,60,20));
					sprintf(data, "_y:%d ",close_y);
					writer_.WriteString(data);
					lcd_.SetRegion(libsc::Lcd::Rect(61,60,59,20));
					sprintf(data, "mid:%d ",grey_midline);
					writer_.WriteString(data);
// 					Dir_pid.SetSetpoint(target_x);
					if ((*onptr)->center.second > close_y && rotate == no)
						rotate = prepare;
					if (!seen)
						seen = true;
					if (not_find_time)
						not_find_time = 0;
					if (rotate == performing
							&& (*onptr)->center.first > grey_midline) {
						action = keep;
					} else {
						rotate = no;
						action = chase;
					}
				}else
				//////////////////////////////////////////
				if (ir_target != NULL&&onptr ==nullptr) //target find
				{
// 					char data[20];
// 					sprintf(data, "I:%d,%d\n", target->center.first,
// 							target->area);
// 					bt.SendStr(data);
					presbeadir=0;
					if (ir_target->area > max_area)
						max_area = (ir_target->area + max_area) / 2;
					target_x = target_slope * max_area + target_intercept;
					if (target_x > 320)
						target_x = 320;
// 					Dir_pid.SetSetpoint(target_x);
					if (ir_target->area > near_area && rotate == no)
						rotate = prepare;
					if (!seen)
						seen = true;
					if (not_find_time)
						not_find_time = 0;
					if (rotate == performing
							&& ir_target->center.first > target_x) {
						action = keep;
					} else {
						rotate = no;
						action = chase;
					}
				} else if (rotate == performing) { //target not find and rotating
					if(presbeadir!=0){
						switch(presbeadir){
						action=avoid;
						case 1:
							rotate=turnleft;
							break;
						case 2:
							rotate=turnright;
							break;
						}
					}else
					if (System::Time() - not_find_time > 800)
						action = rotation;
				} else if (seen) { //target not find but have seen target before
					if (not_find_time == 0) {
						not_find_time = System::Time();
						action = keep;
					} else if (tick - not_find_time > 75) { //target lost for more than 75 ms
						if (rotate == prepare) {
							rotate = performing;
							action = out;
						} else
							action = rotation;
						max_area = 0;
						seen = false;
						Dir_pid.reset();
					}
				} else { //target not find and have not seen target before
					if (finding_time == 0)
						finding_time = System::Time();
					else if (tick - finding_time > 1000) //change to rotate after going foward for 1000ms
						action = rotation;
					else
						action = forward;
				}

				switch (action) {
				case forward:
					L_pid.SetSetpoint(finding_speed);
					R_pid.SetSetpoint(finding_speed);
					break;
				case rotation:
					L_pid.SetSetpoint(rotate_speed);
					R_pid.SetSetpoint(-rotate_speed);
					break;
				case chase:
					int diff;
					diff = Dir_pid.output(target_x, ir_target->center.first);
//					diff = chasing_speed - L_pid.GetSetpoint() + Dir_pid.Calc(target->center.first);
					L_pid.SetSetpoint(chasing_speed - diff);
					R_pid.SetSetpoint(chasing_speed + diff);
					break;
				case out:
					L_pid.SetSetpoint(L_out_speed);
					R_pid.SetSetpoint(R_out_speed);
					break;
				case keep:
					break;
				case avoid:
					switch(rotate){
					case turnleft: /////////need tune
						L_pid.SetSetpoint(30);
						R_pid.SetSetpoint(20);
						break;
					case turnright: /////////need tune
						L_pid.SetSetpoint(20);
						R_pid.SetSetpoint(30);
						break;
					case fastturnleft: /////////need tune
						L_pid.SetSetpoint(50);
						R_pid.SetSetpoint(20);
						break;
					case fastturnright: /////////need tune
						L_pid.SetSetpoint(20);
						R_pid.SetSetpoint(50);
						break;
					case turn180: /////////need tune
						break;
					default:
						break;
					}
					break;
				}
				send(action, state);
				cam->UnlockBuffer();
		lcd_.SetRegion(libsc::Lcd::Rect(0,0,60,20));
		sprintf(data, "rot:%d ",rotate);
		writer_.WriteString(data);
		lcd_.SetRegion(libsc::Lcd::Rect(61,0,59,20));
		sprintf(data, "act:%d ",action);
		writer_.WriteString(data);
		if(notonptr!=nullptr){
			lcd_.SetRegion(libsc::Lcd::Rect(0,20,60,20));
			sprintf(data, "noton:%d ",1);
			writer_.WriteString(data);
			lcd_.SetRegion(libsc::Lcd::Rect(0,80,60,20));
			sprintf(data, "x:%d ",(*notonptr)->center.first);
			writer_.WriteString(data);
			lcd_.SetRegion(libsc::Lcd::Rect(61,80,59,20));
			sprintf(data, "y:%d ",(*notonptr)->center.second);
			writer_.WriteString(data);
		}else{
		lcd_.SetRegion(libsc::Lcd::Rect(0,20,60,20));
		sprintf(data, "noton:%d ",0);
		writer_.WriteString(data);}
		if(onptr!=nullptr){
			lcd_.SetRegion(libsc::Lcd::Rect(61,20,59,20));
			sprintf(data, "on:%d ",1);
			writer_.WriteString(data);
			lcd_.SetRegion(libsc::Lcd::Rect(0,80,60,20));
			sprintf(data, "x:%d ",(*onptr)->center.first);
			writer_.WriteString(data);
			lcd_.SetRegion(libsc::Lcd::Rect(61,80,59,20));
			sprintf(data, "y:%d ",(*onptr)->center.second);
			writer_.WriteString(data);
		}else{
			lcd_.SetRegion(libsc::Lcd::Rect(61,20,59,20));
			sprintf(data, "on:%d ",0);
			writer_.WriteString(data);}
		if(ir_target!=nullptr){
			lcd_.SetRegion(libsc::Lcd::Rect(0,40,60,20));
			sprintf(data, "ir:%d ",1);
			writer_.WriteString(data);
			lcd_.SetRegion(libsc::Lcd::Rect(0,100,60,20));
			sprintf(data, "ir_x:%d ",(*onptr)->center.first);
			writer_.WriteString(data);
			lcd_.SetRegion(libsc::Lcd::Rect(61,100,59,20));
			sprintf(data, "ir_y:%d ",(*onptr)->center.second);
			writer_.WriteString(data);
		}else{
			lcd_.SetRegion(libsc::Lcd::Rect(0,40,60,20));
			sprintf(data, "ir:%d ",0);
			writer_.WriteString(data);}

			}
		}

	}

	return 0;
}
