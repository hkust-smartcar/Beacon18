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
//#include "libbase/k60/pit.h"
#include "libbase/misc_utils_c.h"
#include "image_processing.h"
#include "var.h"
#include "config.h"
#include "debug.h"
#include <libsc/button.h>
//#include <libbase/k60/flash.h>

namespace libbase {
namespace k60 {

Mcg::Config Mcg::GetMcgConfig() {
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}
} // namespace k60
} // namespace libbase

using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

working_mode m = close;
int main() {
	System::Init();

	Led Led0(init_led(0));
	led0 = &Led0;
	Led Led1(init_led(1));
	led1 = &Led1;
	k60::MT9V034 cam_(init_cam());
	cam = &cam_;
	width = cam->GetW();
	height = cam->GetH();
	cam->Start();
	St7735r lcd_(init_lcd());
	lcd = &lcd_;
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd->Clear(Lcd::kBlack);
	LcdTypewriter writer_(init_writer());
	writer = &writer_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;
	Joystick::Config j_config;
	j_config.id = 0;
	j_config.is_active_low = true;
	bool checked = false;
	j_config.dispatcher = [&checked](const uint8_t id, const Joystick::State which) {
		if(run || !checked)
		return;
		switch(which) {
			case Joystick::State::kUp:
			m = image;
			break;
			case Joystick::State::kDown:
			m = close;
			break;
			case Joystick::State::kLeft:
				System::DelayMs(200);
			menu->EnterDebug("leave");
			break;
		}
	};
	Joystick joyStick_(j_config);
	joystick = &joyStick_;

	DebugConsole menu_(joystick, lcd, writer);
	menu_.PushItem("Sobel", &sobel_value, 10);
	menu_.PushItem("LSobel", &line_sobel_value, 10);
	menu_.PushItem("IR", &white, 5);
	menu_.PushItem("IrTime", &ir_timeout, 5);
	menu = &menu_;
	check_cam();
	checked = true;
	/////////var/////////////////
	int start = 0;
	irState = no;

	if (m == word) {
		lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
		writer->WriteString("ir target");
		lcd->SetRegion(Lcd::Rect(0, 30, 160, 15));
		writer->WriteString("o target");
	}

	while (1) {
		if (tick != System::Time() && run) {
			tick = System::Time();
			if (tick - start > 30) {
				start = tick;
				process();
				tick = System::Time();
				cam->UnlockBuffer();
				if (ir_target != NULL && irState == checked) {
					send_coord(PkgType::irTarget);
					led0->SetEnable(true);
				} else
					led0->SetEnable(false);
				if (o_target != NULL && tick - o_find_time > 100) {
					send_coord(PkgType::oTarget);
					led1->SetEnable(true);
				} else {
					o_find_time = 0;
					led1->SetEnable(false);
				}

				if (m != close)
					display_state(m);
				reset_recrod();
//				display_time(start);
			}
		}
	}
	return 0;
}
