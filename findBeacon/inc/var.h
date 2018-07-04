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

struct Record {
	Beacon record;
	uint8_t count = 0;
	Record(Beacon record_) :
			record(record_), count(1) {
	}
};

enum PkgType {
	irTarget = 0, oTarget = 1
};

class distance_recorder {
public:
	distance_recorder() :
			distance(0), start(false) {
	}
	void init() {
		distance = 0;
		start = true;
	}

	uint32_t distance;
	bool start;
};

struct BitConsts {
	uint8_t kSTART = 0xF0;
	uint8_t kEND = 0xFF;
};

enum rotate_state {
	no, prepare, performing
};
enum state_ {
	forward, chase, rotation, out, keep, avoid, approach, backward, stop
};

int chasing_speed = 200;
int finding_speed = 200;
int rotate_speed = 100;
int L_out_speed = 10;
int R_out_speed = 30;

float L_kp = 3;
float L_ki = 0.015;
float L_kd = 0.2;
float R_kp = 4.4;
float R_ki = 0.015;
float R_kd = 0.2;

float Dir_kp = 0.3;
float Dir_ki = 0.0;
float Dir_kd = 0.0;

float avoid_kp = 0.5;
float avoid_ki = 0.01;
float avoid_kd = 0.0;

//Debug
bool move_re = false;
bool stop_re = false;
bool move[4] = { }; //up,down,left,right

//////////////algo parm///////////////////
const float target_slope = 0.009855697800993502;
const float target_intercept = 172.55532972120778;
const float target_slope2 = -1.4209145956223272;
const float target_intercept2 = 98.18294250176507;

int16_t target_x = 0;
state_ action = keep;
bool run = false;
bool seen = false;
uint32_t tick = 0;
const Byte* buf = NULL;
Beacon* ir_target = NULL;
Beacon* ir_record = NULL;
BeaconPackage o_target;
BeaconPackage ir_target2;
std::pair<uint16_t, uint16_t> last_beacon;

const uint16_t width = 320;
const uint16_t height = 240;
const uint16_t numOfPixel = 9600;
uint8_t contrast = 0x40;
uint8_t brightness = 0x00;
uint32_t max_area = 0;
uint32_t near_area = 4500;
uint32_t init_forward_count = 2000;
uint32_t avoid_dead_time = 2000;
uint32_t exit_dead_time = 2000;
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
//IncrementalPidController<int, int>* L_pid = NULL;
//IncrementalPidController<int, int>* R_pid = NULL;
PID* L_pid = NULL;
PID* R_pid = NULL;
PID* Dir_pid = NULL;
PID* avoid_pid = NULL;

#endif /* INC_VAR_H_ */
