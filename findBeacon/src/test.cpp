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
//
//	////////////////Variable init/////////////////
//	tick = System::Time();
//
////	////////////////Main loop////////////////////////
//	while (1) {
//		if (tick != System::Time() ) {
//			tick = System::Time();
//			L_motor->SetPower(400);
//			R_motor->SetPower(400);
//		}
//	}
//	return 0;
//}
//
