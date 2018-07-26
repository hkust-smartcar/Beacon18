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

void sendFloat(float i) {
	Byte out[4];
	memcpy(out, &i, sizeof(float));
	bt->SendBuffer(out, 4);
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
		firstRun = true;
		L_motor->SetClockwise(false);
		R_motor->SetClockwise(true);
		changeSpeedTime = System::Time();
		chases_crash_time = 0;
		led0->SetEnable(1);
		comm->SendStrLiteral("s");
		reset_pid();
		encoder1->Update();
		encoder2->Update();
	}
	if (data[0] == 'S') {
		run = false;
		sendFloat(bMeter->GetVoltage());
		led0->SetEnable(0);
		L_pid->settarget(0);
		R_pid->settarget(0);
		L_motor->SetPower(0);
		R_motor->SetPower(0);
		aaction = stops;
		pastAction = stops;
		if(seen)
		{
			seen = false;
			max_area = 0;
		}
		comm->SendStrLiteral("S");
	}
	return true;
}

void ssend (sstate_ state)
{
	BitConsts a;
	Byte out[4];
	bt->SendBuffer(&a.kSTART, 1);
	Byte size[1] = { 4 };
	bt->SendBuffer(size, 1);
	out[0] = state & 0xFF;
	out[1] = ir_target != NULL?1:0;
	out[2] = tick - o_target.received_time < 200? 1:0;
	out[3] = tick - ir_target2.received_time < 200?1:0;
	bt->SendBuffer(out, 4);
	bt->SendBuffer(&a.kEND, 1);

	if(printLCD)
	{
		char temp[20] = { };
		switch(state)
		{
		case 0:
			sprintf(temp, "s=for");
			break;
		case 1:
			sprintf(temp, "s=chase");
			break;
		case 2:
			sprintf(temp, "s=rot");
			break;
		case 3:
			sprintf(temp, "s=turnR");
			break;
		case 4:
			sprintf(temp, "s=turnL");
			break;
		case 5:
			break;
		case 6:
			sprintf(temp, "s=avoid");
			break;
		case 7:
			sprintf(temp, "s=app");
			break;
		case 8:
			sprintf(temp, "s=back");
			break;
		case 9:
			sprintf(temp, "s=stop");
			break;
		default:
			sprintf(temp, "s=");
		}
		lcd->SetRegion(Lcd::Rect(100, 110, 128, 160));
		writer->WriteString(temp);
	}
}

inline void sendData() {
	BitConsts a;
	Byte out[4];
	bt->SendBuffer(&a.kSTART, 1);
	Byte size[1] = { 4 };
	bt->SendBuffer(size, 1);
	out[0] = aaction & 0xFF;
	out[1] = ir_target != NULL ? 1 : 0;
	out[2] = tick - o_target.received_time < 200 ? 1 : 0;
	out[3] = tick - ir_target2.received_time < 200 ? 1 : 0;
	bt->SendBuffer(out, 4);
	bt->SendBuffer(&a.kEND, 1);
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
	} else if (move[0]) {	//forward
		if (move[2]) {
			L_pid->settarget(chasing_speed);
			R_pid->settarget(chasing_speed * 2);
		} else if (move[3]) {
			L_pid->settarget(chasing_speed * 2);
			R_pid->settarget(chasing_speed);
		} else {
			L_pid->settarget(chasing_speed);
			R_pid->settarget(chasing_speed);
		}
	} else if (move[1]) {	//backward
		if (move[2]) {
			L_pid->settarget(-chasing_speed);
			R_pid->settarget(-(chasing_speed * 2));
		} else if (move[3]) {
			L_pid->settarget(-(chasing_speed * 2));
			R_pid->settarget(-chasing_speed);
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

//void FSM() {
//	int diff;
//	std::pair<uint16_t, uint16_t> p;
//	int speed;
//	sendData();
//	switch (action) {
//	case forward:
//		L_pid->settarget(finding_speed);
//		R_pid->settarget(finding_speed);
//		break;
//	case backward:
//		L_pid->settarget(-finding_speed);
//		R_pid->settarget(-finding_speed);
//		break;
//	case rotation:
//		L_pid->settarget(rotate_speed);
//		R_pid->settarget(-rotate_speed);
//		break;
//	case chase:
//		Dir_pid->settarget(target_x);
//		diff = Dir_pid->output(ir_target->center.first);
//		diff = chasing_speed * diff / 100;
//		L_pid->settarget(chasing_speed - diff);
//		R_pid->settarget(chasing_speed + diff);
//		if (chasing_speed < 150)
//			L_pid->settarget(chasing_speed);
//		break;
//	case stop:
//		L_pid->settarget(0);
//		R_pid->settarget(0);
//		break;
//	case out:
//		if (last_beacon.first < 170) {
//			L_pid->settarget(out_speed);
//			R_pid->settarget(out_speed + out_speed * 0.67);
//		} else {
//			L_pid->settarget(out_speed + out_speed * 0.67);
//			R_pid->settarget(out_speed);
//		}
//		break;
//	case avoid:
//		p = o_target.target->center;
//		if (p.first < 90)
//			avoid_pid->settarget(0);
//		else
//			avoid_pid->settarget(189);
//		diff = avoid_pid->output(p.first);
//		diff = chasing_speed * diff / 100;
//		L_pid->settarget(chasing_speed - diff);
//		R_pid->settarget(chasing_speed + diff);
//		break;
//	case approach:
//		p = ir_target2.target->center;
//		if (p.first < 90)
//			avoid_pid->settarget(10);
//		else
//			avoid_pid->settarget(180);
//		diff = avoid_pid->output(p.first);
//		diff = chasing_speed * diff / 100;
//		L_pid->settarget(chasing_speed - diff);
//		R_pid->settarget(chasing_speed + diff);
//		break;
//	case eixt_rotation:
//		L_pid->settarget(rotate_speed / 2);
//		R_pid->settarget(chasing_speed);
//		break;
//	case keep:
//		break;
//	}
//}

#endif
