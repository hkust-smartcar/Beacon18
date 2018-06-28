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
	St7735r lcd_(init_lcd());
	lcd = &lcd_;
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd->Clear(Lcd::kWhite);
	LcdTypewriter writer_(init_writer());
	writer = &writer_;
	k60::MT9V034 cam_(init_cam());
	cam = &cam_;
	width = cam->GetW();
	height = cam->GetH();
	bool comfirm = false;
	JyMcuBt106 bt_(init_bt(comfirm));
	bt = &bt_;
	JyMcuBt106 comm_(init_comm());
	comm = &comm_;
	int start = 0;
	uint32_t tick = System::Time();
	cam->Start();
	Joystick::Config j_config;
	j_config.id = 0;
	j_config.is_active_low = true;
//	j_config.dispatcher =[](const uint8_t id, const Joystick::State which){
//		char data[20] = {};
//		sprintf(data,"%d",20);
//	};
	Joystick joyStick(j_config);
	bool down_timer = false;
	uint16_t down_time = 0;
	while (!cam->IsAvailable())
		;
	buf = cam->LockBuffer();
	display_greyscale_image();
	while (true) {
		if (joyStick.GetState() == Joystick::State::kIdle && !down_timer)
			continue;
		if (joyStick.GetState() == Joystick::State::kSelect) {
			if (!down_timer) {
				down_timer = true;
				down_time = System::Time();
			}
		} else if (down_timer && System::Time() - down_time > 500) {
			break;
		} else {
			cam->RegSet(0x0C, 0x03);
			cam->UnlockBuffer();
			System::DelayMs(300);
			buf = cam->LockBuffer();
			display_greyscale_image();
			down_timer = false;
		}
	}
	cam->UnlockBuffer();
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd->Clear(Lcd::kWhite);

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
			if (tick % 20 == 0 ) {
				start = tick;
				buf = cam->LockBuffer();
				if (m == image) {
					display_greyscale_image();
					show_avoid_region();
				}
				process();
				Beacon *ir_target_mask = NULL;
				if (irState == checked && System::Time() - find_time > 100)
					ir_target_mask = ir_target;
				if (m == word) {
					lcd->SetRegion(Lcd::Rect(0, 60, 160, 15));
					lcd->FillColor(Lcd::kWhite);
				}
				if (ir_target_mask != NULL) {
					send_coord(PkgType::irTarget);
					if (m == word)
						display_num(PkgType::irTarget);
					else if (m == image)
						display(*ir_target_mask, Lcd::kRed);
					led0->SetEnable(true);
				} else {
					if (m == word) {
						lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
						writer->WriteString("no");
					}
					led0->SetEnable(false);
				}
				if (o_target != NULL) {
					send_coord(PkgType::oTarget);
					if (m == word)
						display_num(PkgType::oTarget);
					else if (m == image)
						display(*o_target, Lcd::kBlue);
					led1->SetEnable(true);
				} else {
					if (m == word) {
						lcd->SetRegion(Lcd::Rect(0, 45, 160, 15));
						writer->WriteString("no");
					}
					led1->SetEnable(false);
				}
				reset_recrod();
				cam->UnlockBuffer();
//				display_time(start);
			}
		}
	}
	return 0;
}
