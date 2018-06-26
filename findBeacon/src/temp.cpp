///*
// * main.cpp
// *
// * Author:Sheldon
// * Copyright (c) 2017-2018 HKUST SmartCar Team
// * Refer to LICENSE for details
// */
//
//#include <cassert>
//#include <cstring>
//#include <stdlib.h>
//#include <string>
//#include <libbase/k60/mcg.h>
//#include <libsc/system.h>
//#include "libsc/joystick.h"
//#include "libsc/battery_meter.h"
//#include "libbase/misc_utils_c.h"
//#include "image_processing.h"
//#include "libutil/misc.h"
//#include "config.h"
//#include "var.h"
//
//namespace libbase {
//namespace k60 {
//
//Mcg::Config Mcg::GetMcgConfig() {
//	Mcg::Config config;
//	config.external_oscillator_khz = 50000;
//	config.core_clock_khz = 150000;
//	return config;
//}
//}
//}
//
//using libsc::System;
//using namespace libsc;
//using namespace libsc::k60;
//using namespace libbase::k60;
//using namespace libutil;
//
//int main() {
//	System::Init();
//	Led Led0(init_led(0));
//	led0 = &Led0;
//	Led Led1(init_led(1));
//	led1 = &Led1;
//	BatteryMeter bMeter_(init_bMeter());
//	bMeter = &bMeter_;
//	St7735r lcd_(init_lcd());
//	lcd = &lcd_;
//	lcd->SetRegion(Lcd::Rect(0, 0, 160, 128));
//	lcd->Clear(Lcd::kWhite);
//	LcdTypewriter writer_(init_writer());
//	writer = &writer_;
//	AlternateMotor motor0(init_motor(0));
//	L_motor = &motor0;
//	AlternateMotor motor1(init_motor(1));
//	R_motor = &motor1;
//	DirEncoder encoder1_(init_encoder(1));
//	encoder1 = &encoder1_;
//	DirEncoder encoder2_(init_encoder(0));
//	encoder2 = &encoder2_;
//	k60::Ov7725 cam_(init_cam());
//	cam = &cam_;
//	cam->Start();
//	//////////////////PID init////////////////////
//	IncrementalPidController<int, int> L_pid_(0, L_kp, L_ki, L_kd);
//	L_pid = &L_pid_;
//	L_pid->SetILimit(0);
//	L_pid->SetOutputBound(-1000, 1000);
//	IncrementalPidController<int, int> R_pid_(0, R_kp, R_ki, R_kd);
//	R_pid = &R_pid_;
//	R_pid->SetILimit(0);
//	R_pid->SetOutputBound(-1000, 1000);
//	PID Dir_pid_(Dir_kp, Dir_ki, Dir_kd);
//	Dir_pid = &Dir_pid_;
//	Dir_pid->errorSumBound = 10000;
//	PID avoid_pid_(avoid_kp, avoid_ki, avoid_kd);
//	avoid_pid = &avoid_pid_;
//	avoid_pid->errorSumBound = 10000;
//
//	////////////////Variable init/////////////////
//	uint32_t tick = System::Time();
//	uint32_t not_find_time = 0;
//	bool seen = false;
//	int finding_time = 0;
//	rotate_state rotate = no;
//	Beacon *ptr = NULL;
//	/////////////////For Dubug////////////////////
//	uint8_t state = 100;
//	uint32_t pid_time = System::Time();
//	JyMcuBt106 bt_(init_bt());
//	bt = &bt_;
//	JyMcuBt106 comm_(init_comm());
//	comm = &comm_;
//	display_bMeter();
////	////////////////Main loop////////////////////////
//	while (1) {
//		if (tick != System::Time() && run) {
//			tick = System::Time();
//			////////////////////PID///////////////////////
//			if (tick % 10 == 0) {
//				uint32_t time_diff = tick - pid_time;
//				encoder1->Update();
//				encoder2->Update();
//				int32_t reading1 = encoder1->GetCount() * 10;
//				int32_t reading2 = encoder2->GetCount() * 10 ;
//				reading1 = reading1 / (int)time_diff;
//				reading2 = reading2 / (int)time_diff;
//				SetPower(GetMotorPower(0) + L_pid->Calc(reading1), 0);
//				SetPower(GetMotorPower(1) + R_pid->Calc(-reading2), 1);
//				pid_time = System::Time();
//			}
//			if (tick % 15 == 0) {
//				///////////////decision making///////////////
////				if (tick - o_target.received_time < 100) {
////
////				}
//				process(seen);
//				tick = System::Time();
//				if (ir_target != NULL) //target find
//				{
//					if (ir_target->area > max_area)
//						max_area = (ir_target->area + max_area) / 2;
//					if (ir_target->area > near_area && rotate == no)
//						rotate = prepare;
//					if (rotate == performing
//							&& ir_target->center.first > target_x) {
//						action = keep;
//					} else {
//						rotate = no;
//						action = chase;
//					}
//					seen = true;
//					not_find_time = 0;
//				} else if (rotate == performing) { //target not find and rotating
//					if (tick - not_find_time > 800)
//						action = rotation;
//				} else if (seen) { //target not find but have seen target before
//					if (not_find_time == 0) {
//						not_find_time = tick;
//						action = keep;
//					} else if (tick - not_find_time > 75) { //target lost for more than 75 ms
//						if (rotate == prepare) {
//							rotate = performing;
//							action = out;
//						} else
//							action = rotation;
//						max_area = 0;
//						seen = false;
//						Dir_pid->reset();
//					}
//				} else { //target not find and have not seen target before
//					if (finding_time == 0)
//						finding_time = tick;
//					else if (tick - finding_time > 1000) //change to rotate after going foward for 1000ms
//						action = rotation;
//					else
//						action = forward;
//				}
//				FSM();
//				send(state);
//			}
//		}
//	}
//
//	return 0;
//}
