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
#include <libsc/k60/ov7725.h>
#include <libsc/dir_encoder.h>
#include "libsc/alternate_motor.h"


using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

struct BeaconPackage{
	Beacon* target = NULL;
	uint16_t received_time = 0;
};

const Byte* buf = NULL;
Beacon* ir_target = NULL;
Beacon* ir_record = NULL;
BeaconPackage o_target;
BeaconPackage ir_target2;

const uint16_t width = 320;
const uint16_t height = 240;
const uint16_t numOfPixel = 9600;
uint8_t contrast = 0x40;
uint8_t brightness = 0x00;
Led* led0 = NULL;
Led* led1 = NULL;
St7735r* lcd = NULL;
LcdTypewriter* writer = NULL;
Ov7725* cam = NULL;
JyMcuBt106* bt = NULL;
JyMcuBt106* comm = NULL;
AlternateMotor *L_motor = NULL;
AlternateMotor *R_motor = NULL;
DirEncoder* encoder1 = NULL;
DirEncoder* encoder2 = NULL;


#endif /* INC_VAR_H_ */
