#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_
#include "var.h"

void SetPower(int speed, int id) {
	bool direction = (speed > 0);
	int power = (speed > 0 ? speed : -speed);
	power = libutil::Clamp<int>(0, power, 1000);
	switch (id) {
	case 0:
		L_motor->SetPower(power);
		L_motor->SetClockwise(!direction);
		break;
	case 1:
		R_motor->SetPower(power);
		R_motor->SetClockwise(direction);
	}
}

int GetMotorPower(int id) {
	switch (id) {
	case 0:
		return L_motor->IsClockwise() ?
				-L_motor->GetPower() : L_motor->GetPower(); //true is forward
		break;
	case 1:
		return R_motor->IsClockwise() ?
				R_motor->GetPower() : -R_motor->GetPower(); //false is forward
		break;
	}
	return 0;
}

void sendInt(int i) {
	Byte out[5];
	out[0] = i < 0 ? 1 : 0;
	i = abs(i);
	out[1] = (i >> 24) & 0xFF;
	out[2] = (i >> 16) & 0xFF;
	out[3] = (i >> 8) & 0xFF;
	out[4] = i & 0xFF;
	bt->SendBuffer(out, 5);
}

std::list<uint8_t> buffer;

inline void BuildTargetPackage() {
	auto it = buffer.begin();
	Byte temp[4];
	float val;
	auto ptr = avoid_pid;
	for (int i = 0; i < 4; i++)
		temp[i] = *(it++);
	memcpy(&val, temp, sizeof(float));
	ptr->kP = val;
	for (int i = 0; i < 4; i++)
		temp[i] = *(it++);
	memcpy(&val, temp, sizeof(float));
	ptr->kI = val;
	for (int i = 0; i < 4; i++)
		temp[i] = *(it++);
	memcpy(&val, temp, sizeof(float));
	ptr->kD = val;
	char data[20] = { };
	int y = 15;
	lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
	y += 15;
	sprintf(data, "kp: %.4f", ptr->kP);
	writer->WriteBuffer(data, 20);
	lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
	y += 15;
	sprintf(data, "ki: %.4f", ptr->kI);
	writer->WriteBuffer(data, 20);
	lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
	y += 15;
	sprintf(data, "kd: %.4f", ptr->kD);
	writer->WriteBuffer(data, 20);
}

inline void BuildBufferPackage() {
	auto it = buffer.begin();
	uint8_t type = *(it++);
	uint8_t x = *(it++);
	uint8_t y = *(it++);
	BeaconPackage* ptr = NULL;
	switch (type) {
	case PkgType::irTarget:
		ptr = &ir_target2;
		break;
	case PkgType::oTarget:
		ptr = &o_target;
		break;
	}
	if (ptr->target == NULL)
		ptr->target = new Beacon();
	ptr->target->center.first = x;
	ptr->target->center.second = y;
	ptr->received_time = System::Time();

}

bool comm_listener(const Byte *data, const size_t size) {
	BitConsts a;
	if (data[0] == a.kSTART)
		buffer.clear();
	else if (data[0] == a.kEND)
		BuildBufferPackage();
//		BuildTargetPackage();
	else
		buffer.push_back(data[0]);
	return true;
}

void reset_pid() {
	L_pid->reset();
	R_pid->reset();
	Dir_pid->reset();
	avoid_pid->reset();
}
bool receiving = false;
bool bt_listener(const Byte *data, const size_t size) {
	BitConsts a;
	if (move_re) {
		move[data[0] - '0'] = true;
		move_re = false;
		return true;
	}
	if (stop_re) {
		move[data[0] - '0'] = false;
		stop_re = false;
		return true;
	}

	if (receiving) {
		if (data[0] == a.kEND) {
			BuildTargetPackage();
			receiving = false;
		} else
			buffer.push_back(data[0]);
		return true;
	}
	if (data[0] == a.kSTART) {
		buffer.clear();
		receiving = true;
		return true;
	}

	if (data[0] == 'm')
		move_re = true;
	if (data[0] == 'p')
		stop_re = true;

	if (data[0] == 's') {
		run = true;
		led1->SetEnable(1);
		comm->SendStrLiteral("s");
		reset_pid();

	}
	if (data[0] == 'S') {
		run = false;
		led1->SetEnable(0);
		L_pid->settarget(0);
		R_pid->settarget(0);
		L_motor->SetPower(0);
		R_motor->SetPower(0);
		comm->SendStrLiteral("S");
	}
	return true;
}

void FSM() {
	int diff;
	std::pair<uint16_t, uint16_t> p;
	int speed;
//	BitConsts a;
//	bt->SendBuffer(&a.kSTART, 1);
//	bt->SendBuffer(0,1);
//	sendInt(action);
//	bt->SendBuffer(&a.kEND, 1);
	switch (action) {
	case forward:
		L_pid->settarget(finding_speed);
		R_pid->settarget(finding_speed);
		break;
	case backward:
		L_pid->settarget(-finding_speed);
		R_pid->settarget(-finding_speed);
		break;
	case rotation:
		L_pid->settarget(rotate_speed);
		R_pid->settarget(-rotate_speed);
		break;
	case chase:
		Dir_pid->settarget(target_x);
		diff = Dir_pid->output(ir_target->center.first);
		diff = chasing_speed * diff / 100;
		L_pid->settarget(chasing_speed - diff);
		R_pid->settarget(chasing_speed + diff);
		if (chasing_speed < 150)
			L_pid->settarget(chasing_speed);
		break;
	case stop:
		L_pid->settarget(0);
		R_pid->settarget(0);
		break;
	case out:
		if (last_beacon.first < 170) {
			L_pid->settarget(out_speed);
			R_pid->settarget(out_speed  + out_speed * 0.67);
		} else {
			L_pid->settarget(out_speed + out_speed * 0.67);
			R_pid->settarget(out_speed);
		}
		break;
	case avoid:
		p = o_target.target->center;
		if (p.first < 90)
			avoid_pid->settarget(0);
		else
			avoid_pid->settarget(189);
		diff = avoid_pid->output(p.first);
		diff = chasing_speed * diff / 100;
		L_pid->settarget(chasing_speed - diff);
		R_pid->settarget(chasing_speed + diff);
		break;
	case approach:
		p = ir_target2.target->center;
		if (p.first < 90)
			avoid_pid->settarget(10);
		else
			avoid_pid->settarget(180);
		diff = avoid_pid->output(p.first);
		diff = chasing_speed * diff / 100;
		L_pid->settarget(chasing_speed - diff);
		R_pid->settarget(chasing_speed + diff);
		break;
	case keep:
		break;
	}
}

inline void send(uint8_t &state) {
	if (action == keep)
		return;
	if (state != action) {
		state = action;
		char data[10];
		sprintf(data, "S:%d\n", state);
		bt->SendStr(data);
	}
}

inline void display_bMeter() {
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
	char data[20] = { };
	sprintf(data, "%f", bMeter->GetVoltage());
	writer->WriteBuffer(data, 20);
}

inline void reControl() {
	if (!(move[0] || move[1] || move[2] || move[3])) {
		L_pid->settarget(0);
		R_pid->settarget(0);
		L_motor->SetPower(0);
		R_motor->SetPower(0);
	} else if (move[0]) {	//forward
		if (move[2]) {
			L_pid->settarget((int) (chasing_speed * 0.67));
			R_pid->settarget(chasing_speed);
		} else if (move[3]) {
			L_pid->settarget(chasing_speed);
			R_pid->settarget((int) (chasing_speed * 0.67));
		} else {
			L_pid->settarget(chasing_speed);
			R_pid->settarget(chasing_speed);
		}
	} else if (move[1]) {	//backward
		if (move[2]) {
			L_pid->settarget(-(int) (chasing_speed * 0.67));
			R_pid->settarget(-chasing_speed);
		} else if (move[3]) {
			L_pid->settarget(-chasing_speed);
			R_pid->settarget(-(int) (chasing_speed * 0.67));
		} else {
			L_pid->settarget(-chasing_speed);
			R_pid->settarget(-chasing_speed);
		}
	} else if (move[2]) {
		L_pid->settarget(-chasing_speed);
		R_pid->settarget(chasing_speed);
	} else if (move[3]) {
		L_pid->settarget(chasing_speed);
		R_pid->settarget(-chasing_speed);
	}
}

#endif
