/*
 * var.h
 *
 *  Created on: Jun 11, 2018
 *      Author: Sheldon
 */

#ifndef INC_VAR_H_
#define INC_VAR_H_

#include "beacon.h"
#include <stdlib.h>
#include "libsc/st7735r.h"
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include "libsc/lcd_typewriter.h"
#include "MT9V034.h"

using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

enum ir_state {
	no = 0, seen, flash, checked
};

enum dir {
	h = 0, v
};

enum scan_mode {
	beacon = 0,
	check_record,
	avoid1,
	avoid2,
	avoid3,
	avoid4,
	avoid5,
	full_screen
};

enum ptr_mode {
	o = 0, oRecord, irRecord, ir_Target, o_Target
};

enum PkgType {
	irTarget = 0, oTarget = 1
};

enum working_mode {
	image = 0, word, close
};

struct regression_line {
	float slope;
	float intercept;
};

struct BitConsts {
	uint8_t kSTART = 0xF0;
	uint8_t kEND = 0xFF;
};

struct point {
	uint8_t x;
	uint8_t y;
	point(uint8_t m_x, uint8_t m_y) :
			x(m_x), y(m_y) {
	}
};

bool run = false;
const Byte* buf = NULL;
Beacon* ir_record = NULL;
Beacon* o_record = NULL;
Beacon* o_target = NULL;
Beacon* ir_target = NULL;
uint16_t width;
uint16_t height;
Led* led0 = NULL;
Led* led1 = NULL;
St7735r* lcd = NULL;
LcdTypewriter* writer = NULL;
MT9V034* cam = NULL;
JyMcuBt106* bt = NULL;
JyMcuBt106* comm = NULL;
Joystick* joystick = NULL;

const uint16_t sobel_value = 140;
ir_state irState = no;
uint32_t find_time = 0;
uint32_t o_find_time = 0;

#endif /* INC_VAR_H_ */
