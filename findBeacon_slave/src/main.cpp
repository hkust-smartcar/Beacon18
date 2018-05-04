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
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
//#include "libsc/joystick.h"
#include "libsc/st7735r.h"
//#include "libsc/battery_meter.h"
//#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>
#include "beacon.h"
#include "libbase/misc_utils_c.h"
#include "image_processing.h"

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

//////////////cam setting////////////////
const uint16_t width = 320;
const uint16_t height = 240;
const uint16_t numOfPixel = width * height / 8;
uint8_t contrast = 0x40;
uint8_t brightness = 0x00;
//////////////algo parm///////////////////
const uint8_t max_beacon = 10;
const uint16_t near_area = 3000;
/////////////state//////////////////////
Beacon* target = NULL;
Beacon last_beacon;

enum rotate_state {
	no, prepare, performing
};

int main() {
	System::Init();

	/////////////////////led init////////////////////
	Led::Config led_config;
	led_config.is_active_low = true;
	led_config.id = 0;
	Led led0(led_config);
	led_config.id = 1;
	Led led1(led_config);
	/////////////////////LCD init///////////////////
	St7735r::Config lcd_config;
	lcd_config.orientation = 1;
	lcd_config.fps = 60;
	St7735r lcd(lcd_config);
	LcdTypewriter::Config writer_config;
	writer_config.lcd = &lcd;
	LcdTypewriter writer(writer_config);
	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd.Clear(Lcd::kWhite);
	////////////////////cam init//////////////////
	Ov7725::Config cam_config;
	cam_config.id = 0;
	cam_config.w = width;
	cam_config.h = height;
	cam_config.contrast = contrast;
	cam_config.brightness = brightness;
	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 cam(cam_config);
	cam.Start();

	////////////////Variable init/////////////////
	uint32_t tick = System::Time();
	uint32_t start;
	uint32_t end;
	uint8_t frame_count = 0;
	Beacon center_record[10];
	bool seen = false;
	rotate_state rotate = no;
	/////////////////For Dubug////////////////////
	bool run = false;
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k9600;
	config.id = 2;
	JyMcuBt106 comm(config);
	comm.SetRxIsr(
			[&lcd,&writer,&led0,&run](const Byte *data, const size_t size) {
				return true;
			});

	////////////////Main loop////////////////////////
	while (1) {
		if (tick != System::Time()) {
			tick = System::Time();
			if (tick % 20 == 0) {
				///////////////////decision//////////////////////
				const Byte* buf = cam.LockBuffer();
//				lcd.SetRegion(Lcd::Rect(0, 0, width, height));
//				lcd.FillBits(0, 0xFFFF, buf, width * height);
				////////////init value///////////////////////
				Beacon b[max_beacon];
				Beacon *beacons = b;
				uint8_t beacon_count = 0;
				target = NULL;
				///////////////process image/////////////////
				process(buf, beacons, beacon_count, seen);
				if (target != NULL) {
					Byte buffer[3];
					Byte* b = buffer;
					buffer[0] = 'x';
					buffer[1] = target->center.first >> 8;
					buffer[2] = target->center.first & 255;
					comm.SendBuffer(b, 3);
					buffer[0] = 'y';
					buffer[1] = target->center.second >> 8;
					buffer[2] = target->center.second & 255;
					comm.SendBuffer(b, 3);

				} else
					comm.SendStrLiteral("n");
				cam.UnlockBuffer();
			}
		}
	}
	return 0;
}
