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
// #include "beacon.h"
#include "libbase/misc_utils_c.h"
// #include "image_processing.h"
#include "camerafilter.h";

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
const uint16_t width = 80;
const uint16_t height = 60;
const uint16_t numOfPixel = width * height / 8;
uint8_t contrast = 0x37;
uint8_t brightness = 0x1E;

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
	/////////////////For Dubug////////////////////
	bool run = false;
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k4800;
	config.id = 2;
	JyMcuBt106 comm(config);
	comm.SetRxIsr(
			[&comm,&lcd,&writer,&led0,&run](const Byte *data, const size_t size) {
				if(data[0] == 's') {
					run = !run;
					led0.Switch();
				}
				return true;
			});
	char sent = ' ';
	////////////////Main loop////////////////////////
	while (1) {
		if (tick != System::Time() && run) {
			tick = System::Time();
			if (tick % 25 == 0) {
				const Byte* buf = cam.LockBuffer();
				
				bool boolbuf[width*height];
				Bytetoboolarray(buf,boolbuf,width,height);
				lcd.SetRegion(Lcd::Rect(0,0,width,height));
				lcd.FillBits(0x0000,0xFFFF,boolbuf,height*width);
				int decision = threepartpixel(boolbuf,width,height);
				lcd.SetRegion(Lcd::Rect(0,60,width,height));
				if(decision==1 && sent != 'R'){
					comm.SendStrLiteral("R");
					sent = 'R';
				}
				if(decision==2 && sent != 'L'){
					comm.SendStrLiteral("L");
					sent = 'L';
				}
				if(decision==3 && sent != 'F'){
					comm.SendStrLiteral("F");
					sent = 'F';
				}
				cam.UnlockBuffer();
			}
		}
	}
	return 0;
}
