/*
 * main.cpp
 *
 * Author:Sheldon
 * Copyright (c) 2017-2018 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <string>
#include <stdlib.h>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/dir_encoder.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
//#include "libsc/joystick.h"
#include "libsc/st7735r.h"
//#include "libsc/battery_meter.h"
#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>

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

const uint16_t width = 320;
const uint16_t height = 240;


int main() {
	System::Init();
	//led init
	Led::Config led_config;
	led_config.is_active_low = true;
	led_config.id = 0;
	Led led0(led_config);
	led_config.id = 1;
	Led led1(led_config);

	St7735r::Config lcd_config;
	lcd_config.orientation = 0;
	lcd_config.fps = 20;
	St7735r lcd(lcd_config);

	LcdTypewriter::Config writer_config;
	writer_config.lcd = &lcd;
	LcdTypewriter writer(writer_config);

	uint32_t tick = System::Time();
	bool c_start = false;
	int contrast = 0x40;
	int brightness = 0x00;

	Ov7725::Config cam_config;
	cam_config.id = 0;
	cam_config.w = width;
	cam_config.h = height;
	cam_config.contrast = contrast;
	cam_config.brightness = brightness;
	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 cam(cam_config);


	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 1;
	config.tx_buf_size = 2; //change this to 1 if working with large image size
	JyMcuBt106 bt(config);
	config.id = 0;
	JyMcuBt106 bt2(config);
	bt.SetRxIsr(
			[&lcd,&writer,&led0,&c_start,&led1,&brightness,&cam,&bt,&contrast](const Byte *data, const size_t size) {
				if(data[0] =='c') {
					led0.Switch();
					c_start = true;
						cam.Start();
						char temp[20] = {};
						sprintf(temp,"%d\n%d\n",width,width * height /8);
						bt.SendStr(temp);
				}
				if(data[0]=='s') {
					c_start = false;
					led0.SetEnable(0);
					led1.SetEnable(0);
				}

				if(data[0] == 'q') {
					brightness+= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				if(data[0] == 'w') {
					brightness-= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				if(data[0] == 'e') {
					contrast+= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				if(data[0] == 'r') {
					contrast-= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				return true;
			});

	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd.Clear(Lcd::kWhite);

	while (1) {
		if (tick != System::Time()) {
			tick = System::Time();
			if (c_start && tick % 450 == 0) {
				const Byte* buf = cam.LockBuffer();
				bt.SendBuffer(buf, width * height / 16);
				bt2.SendBuffer(buf + width * height / 16, width * height / 16);
				cam.UnlockBuffer();
			}
		}
	}
	return 0;
}
