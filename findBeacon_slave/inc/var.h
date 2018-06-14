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

const Byte* buf = NULL;
Beacon* ir_target = NULL;
Beacon* ir_record = NULL;
Beacon* o_record = NULL;
Beacon* o_target = NULL;

uint16_t width;
uint16_t height;
Led* led0 = NULL;
Led* led1 = NULL;
St7735r* lcd = NULL;
LcdTypewriter* writer = NULL;
MT9V034* cam = NULL;
JyMcuBt106* bt = NULL;
JyMcuBt106* comm = NULL;

uint16_t sobel_value = 90;


#endif /* INC_VAR_H_ */
