///*
// * avoid.cpp
// *
// *  Created on: Jun 21, 2018
// *      Author: Sheldon
// */
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
//} // namespace k60
//} // namespace libbase
//
//using libsc::System;
//using namespace libsc;
//using namespace libsc::k60;
//using namespace libbase::k60;
//using namespace libutil;
//
////////////////algo parm///////////////////
//const uint16_t near_area = 4000;
//const float target_slope = -1.4209145956223272;
//const float target_intercept = 98.18294250176507;
//
//int main() {
//	System::Init();
//	Led Led0(init_led(0));
//	led0 = &Led0;
//	Led Led1(init_led(1));
//	led1 = &Led1;
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
//	BatteryMeter bMeter_(init_bMeter());
//	bMeter = &bMeter_;
//	//////////////////PID init////////////////////
//	IncrementalPidController<int, int> L_pid_(0, L_kp, L_ki, L_kd);
//	L_pid = &L_pid_;
//	L_pid->SetILimit(0);
//	L_pid->SetOutputBound(-1000, 1000);
//	IncrementalPidController<int, int> R_pid_(0, R_kp, R_ki, R_kd);
//	R_pid = &R_pid_;
//	R_pid->SetILimit(0);
//	R_pid->SetOutputBound(-1000, 1000);
//	PID Dir_pid_(Dir_kp, Dir_ki, Dir_kp);
//	Dir_pid = &Dir_pid_;
//	Dir_pid->errorSumBound = 10000;
//	////////////////Variable init/////////////////
//	uint32_t tick = System::Time();
//	uint32_t not_find_time = 0;
//	bool seen = false;
//	int finding_time = 0;
//	rotate_state rotate = no;
//	uint32_t max_area = 0;
//	/////////////////For Dubug////////////////////
//	uint8_t state = 100;
//	JyMcuBt106 bt_(init_bt());
//	bt = &bt_;
//	JyMcuBt106 comm_(init_comm());
//	comm = &comm_;
//	uint32_t pid_time = System::Time();
//	////////////////Main loop////////////////////////
//	while (1) {
//		if (tick != System::Time()) {
//			tick = System::Time();
//			if (tick % 10 == 0) {
//				uint32_t time_diff = System::Time() - pid_time;
//				encoder1->Update();
//				encoder2->Update();
//				int32_t reading1 = encoder1->GetCount() * 10 / time_diff;
//				int32_t reading2 = encoder2->GetCount() * 10 / time_diff;
//				SetPower(GetMotorPower(0) + L_pid->Calc(reading1), 0);
//				SetPower(GetMotorPower(1) + R_pid->Calc(-reading2), 1);
//				pid_time = System::Time();
//			}
//
//			if (tick % 20 == 0 && run) {
//				if (tick - o_target.received_time < 100) {
//					ptr = o_target.target;
//					int x = ptr->center.first;
//					int y = ptr->center.second;
//					target_x = target_slope * y + target_intercept;
//					int diff = Dir_pid->output(target_x, x);
//					if (diff < 0)
//						diff = 0;
//					L_pid->SetSetpoint(L_pid->GetSetpoint() - diff);
//					R_pid->SetSetpoint(R_pid->GetSetpoint() + diff);
////					char data[20];
////					sprintf(data, "I:%d,%d\n", ptr->center.first,
////							ptr->center.second);
////					bt->SendStr(data);
////					if (ptr->center.second > 50) {
////						L_pid->SetSetpoint(0);
////						R_pid->SetSetpoint(0);
////						while (run) {
////						}
////						continue;
////					}
//				} else {
//					L_pid->SetSetpoint(finding_speed);
//					R_pid->SetSetpoint(finding_speed);
//				}
//			}
//		}
//	}
//	return 0;
//}
//
