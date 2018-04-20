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
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/dir_encoder.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
//#include "libsc/joystick.h"
#include "libsc/st7735r.h"
//#include "libsc/battery_meter.h"
//#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>
//#include "libsc/alternate_motor.h"
#include "beacon.h"
#include "libbase/misc_utils_c.h"

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

const uint16_t width = 80;
const uint16_t height = 60;
const uint16_t numOfPixel = width * height / 8;

uint8_t contrast = 0x40;
uint8_t brightness = 0x00;
const uint8_t x_range = 10;
const uint8_t y_range = 10;

int16_t getX(uint16_t m_pos, int8_t m_bit_pos) {
	int16_t x = (m_pos + 1) * 8 % width - m_bit_pos - 1;
	if (x < 0)
		x = width - m_bit_pos - 1;
	return x;
}

int16_t getY(uint16_t m_pos) {
	int16_t y = m_pos * 8 / width;
	return y;
}

Beacon scan(const Byte* buf, uint16_t m_pos, int8_t m_bit_pos) {

	int16_t pos = m_pos;
	int8_t bit_pos = m_bit_pos;
	int16_t x = getX(pos, bit_pos);
	int16_t y = getY(pos);
	Beacon beacon(x, y);

	//scan left
	uint8_t x_count = 0;
	uint8_t y_count = 0;
	uint8_t i = 0;
	bool all_black = true;
	while (x_count < x_range) {
		while (y_count < y_range) {
			int16_t temp_pos = pos + width * i / 8;
			if (temp_pos >= numOfPixel)
				break;
			if (!GET_BIT(buf[temp_pos], bit_pos)) {
				int16_t temp_y = y + i;
				all_black = false;
				beacon.count++;
				if (x < beacon.left_x)
					beacon.left_x = x;
				if (temp_y > beacon.lower_y)
					beacon.lower_y = temp_y;
				y_count = 0;
			} else
				y_count++;
			i++;
		}

		if (all_black)
			x_count++;
		else
			x_count = 0;

		if (++bit_pos > 7) {
			bit_pos = 0;
			--pos;
		}
		if (pos < 0)
			break;
		x = getX(pos, bit_pos);
		y_count = 0;
		i = 0;
		all_black = true;
		if (pos * 8 / width != y)
			break;
	}

//	//scan right
	pos = m_pos;
	bit_pos = m_bit_pos;
	if (--bit_pos < 0) {
		bit_pos = 7;
		++pos;
	}
	x_count = 0;
	y_count = 0;
	i = 0;
	while (x_count < x_range) {
		while (y_count < y_range) {
			int16_t temp_pos = pos + width * i / 8;
			if (temp_pos >= numOfPixel)
				break;
			if (!GET_BIT(buf[temp_pos], bit_pos)) {
				int16_t temp_y = y + i;
				all_black = false;
				beacon.count++;
				if (x > beacon.right_x)
					beacon.right_x = x;
				if (temp_y > beacon.lower_y)
					beacon.lower_y = temp_y;
				y_count = 0;
			} else
				y_count++;
			i++;
		}

		if (all_black)
			x_count++;
		else
			x_count = 0;

		if (--bit_pos < 0) {
			bit_pos = 7;
			++pos;
		}
		x = getX(pos, bit_pos);
		y_count = 0;
		i = 0;
		all_black = true;
		if (pos * 8 / width != y)
			break;
	}
	return beacon;
}

Beacon process(const Byte* buf) {
	uint16_t pos = 0;
	int8_t bit_pos = 8;
	for (uint16_t y = 0; y < height; y++) {
		for (uint16_t x = 0; x < width; ++x) {
			if (--bit_pos < 0) {
				bit_pos = 7;
				++pos;
			}
			if (!GET_BIT(buf[pos], bit_pos)) {
				return scan(buf, pos, bit_pos);
			}
		}
	}
	return Beacon(0, 0);
}

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
	lcd_config.orientation = 1;
	lcd_config.fps = 60;
	St7735r lcd(lcd_config);

	LcdTypewriter::Config writer_config;
	writer_config.lcd = &lcd;
	LcdTypewriter writer(writer_config);

	Ov7725::Config cam_config;
	cam_config.id = 0;
	cam_config.w = width;
	cam_config.h = height;
	cam_config.contrast = contrast;
	cam_config.brightness = brightness;
	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 cam(cam_config);
	cam.Start();

	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd.Clear(Lcd::kWhite);

	uint32_t tick = System::Time();
//	uint32_t start;
//	uint32_t end;
	while (1) {
		if (tick != System::Time()) {
			tick = System::Time();
			if (tick % 5 == 0) {
//				start = System::Time();
				const Byte* buf = cam.LockBuffer();
				lcd.SetRegion(Lcd::Rect(0, 0, 80, 60));
				lcd.FillBits(0, 0xFFFF, buf, width * height);
				Beacon beacon = process(buf);
				lcd.SetRegion(Lcd::Rect(beacon.left_x, 0, 1, 60));
				lcd.FillColor(lcd.kGreen);
				lcd.SetRegion(Lcd::Rect(beacon.right_x, 0, 1, 60));
				lcd.FillColor(lcd.kGreen);
				lcd.SetRegion(Lcd::Rect(0, beacon.upper_y, 80, 1));
				lcd.FillColor(lcd.kGreen);
				lcd.SetRegion(Lcd::Rect(0, beacon.lower_y, 80, 1));
				lcd.FillColor(lcd.kGreen);
				cam.UnlockBuffer();
//				end = System::Time();
//				char data[10] ={};
//				sprintf(data,"%d",end - start);
//				lcd.SetRegion(Lcd::Rect(0, 0, 80, 15));
//				writer.WriteBuffer(data,10);

			}
		}
	}
	return 0;
}
