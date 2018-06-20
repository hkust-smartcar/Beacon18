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
const int chasing_speed = 100;
const int finding_speed = 70;
const int rotate_speed = 120;
const int L_out_speed = 120;
const int R_out_speed = 70;

enum rotate_state {
	no, prepare, performing
};
enum state_ {
	forward, chase, rotation, out, keep
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
	Pit pit_(init_pid_loop());
	pit = &pit_;
	////////////////Variable init/////////////////
	uint32_t tick = System::Time();
	uint32_t not_find_time = 0;
	bool seen = false;
	int finding_time = 0;
	rotate_state rotate = no;
	state_ action = forward;
	uint16_t target_x = target_intercept;
	uint32_t max_area = 0;
	/////////////////For Dubug////////////////////
	uint8_t state = 100;
	JyMcuBt106 bt_(init_bt());
	bt = &bt_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;

	lcd->SetRegion(Lcd::Rect(0,0,160,15));
	char data[20] ={};
	sprintf(data,"%f",bMeter->GetVoltage());
	writer->WriteBuffer(data,20);

	////////////////Main loop////////////////////////
	while (1) {
		if (tick != System::Time() && run) {
			tick = System::Time();
			if (tick % 20 == 0) {
//				char data[20];
//				sprintf(data, "I:%d,%d\n", L_pid->GetSetpoint(),
//						R_pid->GetSetpoint());
//				bt->SendStr(data);
				///////////////decision making///////////////
				buf = cam->LockBuffer();
				////////////init value///////////////////////
				ir_target = NULL;
				///////////////process image/////////////////
				process(seen);
				if (ir_target != NULL) //ir_target find
				{
					if (ir_target->area > max_area)
						max_area = (ir_target->area + max_area) / 2;
					target_x = target_slope * max_area + target_intercept;
					if (target_x > 320)
						target_x = 320;
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
				} else if (rotate == performing) { //ir_target not find and rotating
					if (System::Time() - not_find_time > 800)
						action = rotation;
				} else if (seen) { //ir_target not find but have seen ir_target before
					if (not_find_time == 0) {
						not_find_time = System::Time();
						action = keep;
					} else if (tick - not_find_time > 75) { //ir_target lost for more than 75 ms
						if (rotate == prepare) {
							action = out;
						} else {
							rotate = performing;
							action = rotation;
						}
						max_area = 0;
						seen = false;
						Dir_pid->reset();
					}
				} else { //ir_target not find and have not seen ir_target before
					if (finding_time == 0)
						finding_time = System::Time();
					else if (tick - finding_time > 1000) //change to rotate after going foward for 1000ms
						action = rotation;
					else
						action = forward;
				}

				switch (action) {
				case forward:
					L_pid->SetSetpoint(finding_speed);
					R_pid->SetSetpoint(finding_speed);
					break;
				case rotation:
					L_pid->SetSetpoint(rotate_speed);
					R_pid->SetSetpoint(-rotate_speed);
					break;
				case chase:
					int diff;
					diff = Dir_pid->output(target_x, ir_target->center.first);
					L_pid->SetSetpoint(chasing_speed - diff);
					R_pid->SetSetpoint(chasing_speed + diff);
					break;
				case out:
					L_pid->SetSetpoint(L_out_speed);
					R_pid->SetSetpoint(R_out_speed);
					break;
				case keep:
					break;
				}
				send(action, state);
				cam->UnlockBuffer();
			}
		}
	}
	return 0;
}
