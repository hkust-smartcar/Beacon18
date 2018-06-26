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
#include "libutil/incremental_pid_controller.h"
#include "pid.h"
#include "libbase/k60/pit.h"
#include "libutil/misc.h"

using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;

struct BeaconPackage {
	Beacon* target = NULL;
	uint32_t received_time = 0;
};

enum PkgType {
	irTarget = 0, oTarget = 1
};
struct BitConsts {
	uint8_t kSTART = 0xF0;
	uint8_t kEND = 0xFF;
};

enum rotate_state {
	no, prepare, performing
};
enum state_ {
	forward, chase, rotation, out, keep,avoid
};

int chasing_speed = 50;
int finding_speed = 30;
int rotate_speed = 50;
int L_out_speed = 50;
int R_out_speed = 50;

float L_kp = 2.5;
float L_ki = 0.02;
float L_kd = 0;
float R_kp = 2.5;
float R_ki = 0.02;
float R_kd = 0;
float Dir_kp = 0.5;
float Dir_ki = 0.0;
float Dir_kd = 0.05;
float avoid_kp = 0.5;
float avoid_ki = 0.0;
float avoid_kd = 0.05;

//////////////algo parm///////////////////
const float target_slope = 0.009855697800993502;
const float target_intercept = 172.55532972120778;
const float target_slope2 = -1.4209145956223272;
const float target_intercept2 = 98.18294250176507;

int16_t target_x = 0;
state_ action = keep;
bool run = false;
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
uint32_t max_area = 0;
uint32_t near_area = 3000;
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
BatteryMeter* bMeter = NULL;
IncrementalPidController<int, int>* L_pid = NULL;
IncrementalPidController<int, int>* R_pid = NULL;
PID* Dir_pid = NULL;
PID* avoid_pid = NULL;

#endif /* INC_VAR_H_ */
