/*
 * test.cpp
 *
 *  Created on: Jun 25, 2018
 *      Author: Sheldon
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

int main() {
	System::Init();
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
	j_config.dispatcher = [](const uint8_t id, const Joystick::State which) {
		char data[20] = {};
		sprintf(data,"%d",20);
	};
	Joystick joyStick(j_config);
	//////////////////PID init////////////////////
	PID L_pid_(L_kp, L_ki, L_kd, 1000, -1000);
	L_pid = &L_pid_;
	L_pid->errorSumBound = 10000;
	PID R_pid_(R_kp, R_ki, R_kd, 1000, -1000);
	R_pid = &R_pid_;
	R_pid->errorSumBound = 10000;

	PID Dir_pid_(Dir_kp, Dir_ki, Dir_kd, 500, -500);
	Dir_pid = &Dir_pid_;
	Dir_pid->errorSumBound = 10000;
	PID avoid_pid_(avoid_kp, avoid_ki, avoid_kd, 100, -100);
	avoid_pid = &avoid_pid_;
	avoid_pid->errorSumBound = 10000;

	////////////////Variable init/////////////////
	tick = System::Time();
	uint32_t not_find_time = 0;
	int finding_time = 0;
	rotate_state rotate = no;
	Beacon *ptr = NULL;
	uint32_t pid_time = System::Time();
	uint32_t process_time = System::Time();
	int L_count = 0;
	int R_count = 0;
	/////////////////For Dubug////////////////////
	uint8_t state = 100;
	JyMcuBt106 bt_(init_bt());
	bt = &bt_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;
	display_bMeter();

	reset_pid();

//	////////////////Main loop////////////////////////
	while (1) {
		if (tick != System::Time() && run) {
			tick = System::Time();
			if (tick - pid_time >= 10) {
				uint32_t time_diff = tick - pid_time;
				encoder1->Update();
				encoder2->Update();
				L_count = encoder1->GetCount() * 50 / (int) time_diff;
				R_count = encoder2->GetCount() * 50 / (int) time_diff;
				SetPower(L_pid->output(L_count), 0);
				SetPower(R_pid->output(-R_count), 1);
				pid_time = System::Time();
			}
//			if (tick - ir_target2.received_time < 100) {
//				BitConsts a;
//				bt->SendBuffer(&a.kSTART, 1);
//				sendInt(ir_target2.target->center.first);
//				sendInt(ir_target2.target->center.second);
//				bt->SendBuffer(&a.kEND, 1);
//			}
			if (tick - process_time >= 25) {
				process_time = tick;
				///////////////decision making///////////////
				//				if (tick - o_target.received_time < 50 && action != rotation) {
				//					action = avoid;
				//					FSM();
				//					continue;
				////					ptr = o_target.target;
				////					char data[20] = {};
				////					lcd->SetRegion(Lcd::Rect(0,0,160,15));
				////					sprintf(data,"%d : %d", ptr->center.first, ptr->center.second);
				////					writer->WriteBuffer(data,20);
				//				}
				//				if (tick - ir_target2.received_time < 50) {
				//					action = approach;
				//					FSM();
				//					continue;
				////					ptr = o_target.target;
				////					char data[20] = {};
				////					lcd->SetRegion(Lcd::Rect(0,0,160,15));
				////					sprintf(data,"%d : %d", ptr->center.first, ptr->center.second);
				////					writer->WriteBuffer(data,20);
				//				}
				process();
				tick = System::Time();
				if (ir_target != NULL) {	//target find
					led0->SetEnable(1);
					not_find_time = 0;
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
					if (action == rotation
							&& ir_target->center.first > target_x)
						action = keep;
					else
						action = chase;
				} else if (seen) { //target not find but have seen target before
					if (not_find_time == 0) {
						not_find_time = tick;
						action = keep;
					} else if (tick - not_find_time > 400) { //target lost for more than 75 ms
						//						action = backward;
						led0->SetEnable(0);
						seen = false;
						max_area = 0;
					}
				} else { //target not find and have not seen target before
					led0->SetEnable(0);
					action = rotation;
//					if (finding_time == 0)
//						finding_time = tick;
//					else if (tick - finding_time > 1000) //change to rotate after going foward for 1000ms
//						action = rotation;
//					else
//						action = forward;
				}
				FSM();
				//	send (state);
			}
		}
	}
	return 0;
}

