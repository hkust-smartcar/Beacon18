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
//#include <libsc/dir_encoder.h>
//#include <libsc/led.h>
//#include <libsc/k60/jy_mcu_bt_106.h>
////#include "libsc/joystick.h"
//#include "libsc/st7735r.h"
//#include "libsc/battery_meter.h"
////#include "libbase/k60/pit.h"
//#include "libsc/lcd_typewriter.h"
//#include <libsc/k60/ov7725.h>
//#include "libsc/alternate_motor.h"
//#include "beacon.h"
//#include "libbase/misc_utils_c.h"
//#include "libutil\incremental_pid_controller.h"
//#include "pid.h"
//#include "image_processing.h"
//#include "libutil/misc.h"
//#include "ir.h"
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
//
//	/////////////////////led init////////////////////
//	Led::Config led_config;
//	led_config.is_active_low = true;
//	led_config.id = 0;
//	Led led0(led_config);
//	led_config.id = 1;
//	Led led1(led_config);
//
//	BatteryMeter::Config bConfig;
//	bConfig.voltage_ratio = 0.4;
//	BatteryMeter bMeter(bConfig);
//	/////////////////////LCD init///////////////////
//	St7735r::Config lcd_config;
//	lcd_config.orientation = 1;
//	lcd_config.fps = 60;
//	St7735r lcd(lcd_config);
//	LcdTypewriter::Config writer_config;
//	writer_config.lcd = &lcd;
//	LcdTypewriter writer(writer_config);
//	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
//	lcd.Clear(Lcd::kWhite);
//	///////////////////Motor init///////////////////
//
//	////////////////ir/////////////////////
//	IR_recevier ir1(libbase::k60::Pin::Name::kPtd12);
//	IR_recevier ir2(libbase::k60::Pin::Name::kPtd13);
//	IR_recevier ir3(libbase::k60::Pin::Name::kPtd14);
//	IR_recevier ir4(libbase::k60::Pin::Name::kPtd15);
//
//	////////////////Variable init/////////////////
//	uint32_t tick = System::Time();
//	char data[20] = { };
//	////////////////Main loop////////////////////////
//	while (1) {
//		if (tick != System::Time()) {
//			tick = System::Time();
//			////////////////////PID///////////////////////
//			if (tick % 10 == 1) {
//				lcd.SetRegion(Lcd::Rect(0, 0, 160, 15));
//				sprintf(data, "ir1: %d", ir1.get_pulse());
//				writer.WriteBuffer(data, 20);
//				lcd.SetRegion(Lcd::Rect(0, 15, 160, 15));
//				sprintf(data, "ir2: %d", ir2.get_pulse());
//				writer.WriteBuffer(data, 20);
//				lcd.SetRegion(Lcd::Rect(0, 30, 160, 15));
//				sprintf(data, "ir3: %d", ir3.get_pulse());
//				writer.WriteBuffer(data, 20);
//				lcd.SetRegion(Lcd::Rect(0, 45, 160, 15));
//				sprintf(data, "ir4: %d", ir4.get_pulse());
//				writer.WriteBuffer(data, 20);
//// 				lcd.SetRegion(Lcd::Rect(0, 16, 160, 15));
//// 				if(ir1.getState())
//// 					sprintf(data, "True");
//// 				else
//// 					sprintf(data, "False");
//// 				writer.WriteBuffer(data, 20);
//			}
//		}
//	}
//	return 0;
//}
