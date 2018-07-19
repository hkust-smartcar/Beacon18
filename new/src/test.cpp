///*
// * test.cpp
// *
// *  Created on: Jun 25, 2018
// *      Author: Sheldon
// */
//
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
//#include <math.h>
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
//const float car_width = 15.7;
//const float radius = car_width / 2;
//const float conv = 58.6;
//#define PI 3.14159265
//
//enum direction {
//	front, fLeft, fRight, back, bLeft, bRight, rLeft, rRight, s
//};
//struct encoder_record {
//	int long_time = 0;
//	int short_time = 0;
//};
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
////	k60::Ov7725 cam_(init_cam());
////	cam = &cam_;
////	cam->Start();
////	Joystick::Config j_config;
////	j_config.id = 0;
////	Joystick joyStick(j_config);
//	//////////////////PID init////////////////////
//	PID L_pid_(L_kp, L_ki, L_kd, 1000, -1000);
//	L_pid = &L_pid_;
//	L_pid->errorSumBound = 100000;
//	PID R_pid_(R_kp, R_ki, R_kd, 1000, -1000);
//	R_pid = &R_pid_;
//	R_pid->errorSumBound = 100000;
//
//	PID Dir_pid_(Dir_kp, Dir_ki, Dir_kd, 500, -500);
//	Dir_pid = &Dir_pid_;
//	Dir_pid->errorSumBound = 10000;
//	PID avoid_pid_(avoid_kp, avoid_ki, avoid_kd, 500, -500);
//	avoid_pid = &avoid_pid_;
//	avoid_pid->errorSumBound = 10000;
//
//	////////////////Variable init/////////////////
//	tick = System::Time();
//	uint32_t not_find_time = 0;
//	int finding_time = 0;
//	Beacon *ptr = NULL;
//	uint32_t pid_time = System::Time();
//	uint32_t process_time = System::Time();
//	distance_recorder avoid_counter;
//	distance_recorder exit_counter;
//	distance_recorder out_counter;
//	uint32_t record_time = 0;
//	int L_count = 0;
//	int R_count = 0;
//	/////////////////For Dubug////////////////////
//	uint8_t state = 100;
//	JyMcuBt106 bt_(init_bt());
//	bt = &bt_;
//	JyMcuBt106 comm_(init_comm());
//	comm = &comm_;
//	display_bMeter();
//	encoder_record left;
//	encoder_record right;
//	int32_t update_time = 0;
//	float dist_record = 0.0f;
//	float angle_record = 0.0f;
//	direction dir = s;
//	float out_width = 0.0f;
//	float in_width = 0.0f;
//	float angle = 0.0f;
//	float a = 0.0f;
//	float b = 0.0f;
//	float dist = 0.0f;
//
//	reset_pid();
//
////	////////////////Main loop////////////////////////
//	while (1) {
//		if (tick != System::Time() && run) {
//			tick = System::Time();
//			if (tick % 11 == 0)
//				reControl();
//
//			if (tick - pid_time >= 10) {
//				uint32_t time_diff = tick - pid_time;
//				encoder1->Update();
//				encoder2->Update();
//				L_count = encoder1->GetCount();
//				R_count = encoder2->GetCount();
//				left.long_time += L_count;
//				right.long_time -= R_count;
//				left.short_time += L_count;
//				right.short_time -= R_count;
//				if (L_pid->getTarget() == 0)
//					SetPower(0, 0);
//				else {
//					L_count *= 50 / (int) time_diff;
//					SetPower(L_pid->output(L_count), 0);
//				}
//				if (R_pid->getTarget() == 0)
//					SetPower(0, 1);
//				else {
//					R_count *= 50 / (int) time_diff;
//					SetPower(R_pid->output(-R_count), 1);
//				}
//				pid_time = System::Time();
//			}
//			if (tick - update_time > 200) {
//				direction newdir = s;
//				int speed_diff = left.short_time - right.short_time;
//				if (left.short_time == 0 && right.short_time == 0)
//					newdir = s;
//				else if (left.short_time > 0) {
//					if (right.short_time > 0) {
//						if (speed_diff > 100)
//							newdir = fLeft;
//						else if (speed_diff < -100)
//							newdir = fRight;
//						else
//							newdir = front;
//					} else
//						newdir = rRight;
//				} else if (left.short_time < 0) {
//					if (right.short_time < 0) {
//						if (speed_diff > 100)
//							newdir = bLeft;
//						else if (speed_diff < -100)
//							newdir = bRight;
//						else
//							newdir = back;
//					} else
//						newdir = rLeft;
//				}
//				if ((newdir != s && dir != newdir) || angle > 170
//						|| angle < -170) {
//					if(dir == fLeft || dir == dir == bLeft)
//						dist_record -= dist;
//						else
//						dist_record += dist;
//					angle_record += angle;
//					if (angle_record > 360)
//						angle_record -= 360;
//					if (angle_record < -360)
//						angle_record += 360;
//					left.long_time = 0;
//					right.long_time = 0;
//					dir = newdir;
//				}
//				left.short_time = 0;
//				right.short_time = 0;
//				update_time = tick;
//			}
//			if (tick - record_time >= 25) {
//				out_width = left.long_time / conv;
//				in_width = right.long_time / conv;
//				if (dir == rLeft || dir == rRight) {
//					a = 180 - (abs(out_width) / radius * 180 / PI);
//					b = abs(in_width) / radius * 180 / PI;
//					b = (180 - a - b) / 2;
//					angle = 180 - b - a;
//					dist = sin(b * PI / 180) * radius;
//					if (dir == rLeft) {
//						angle = -angle;
//						dist = -dist;
//					}
//				} else {
//					a = (out_width - in_width) / car_width;
//					if (a == 0)
//						dist = out_width;
//					else
//						dist = 2 * (in_width / a + car_width) * sin(a * 0.5);
//					angle = a * 180 / PI;
//				}
//				if (L_pid->getTarget() == 0 && R_pid->getTarget() == 0) {
//					char data[20] = { };
//					lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
//					sprintf(data, "L: %d", left.long_time);
//					writer->WriteBuffer(data, 20);
//					lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
//					sprintf(data, "R: %d", right.long_time);
//					writer->WriteBuffer(data, 20);
//					lcd->SetRegion(Lcd::Rect(0, 30, 160, 15));
//					char data2[20] = { };
//					sprintf(data2, "A: %0.2f , %0.2f", angle, angle_record);
//					writer->WriteBuffer(data2, 20);
//					lcd->SetRegion(Lcd::Rect(0, 45, 160, 15));
//					sprintf(data2, "d: %0.2f, %0.2f", dist, dist_record);
//					writer->WriteBuffer(data2, 20);
//				}
//				record_time = System::Time();
//			}
//		}
//	}
//	return 0;
//}
//
