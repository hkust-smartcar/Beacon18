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

std::list<uint8_t> buffer;
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
	else
		buffer.push_back(data[0]);
	return true;
}

bool bt_listener(const Byte *data, const size_t size) {
	if (data[0] == 's') {
		run = true;
		led0->SetEnable(1);
		comm->SendStrLiteral("s");
		// pit->SetEnable(true);
	}
	if (data[0] == 'S') {
		run = false;
		led0->SetEnable(0);
		L_motor->SetPower(0);
		R_motor->SetPower(0);
		// pit->SetEnable(false);
		comm->SendStrLiteral("S");
	}
	return true;
}

inline void send_data() {
	char data[20];
	sprintf(data, "I:%d,%d\n", ir_target->center.first, ir_target->area);
	bt->SendStr(data);
}

void FSM() {
	int diff;
	std::pair<uint16_t, uint16_t> p;
	int speed;
	char data[20];
	switch (action) {
	case forward:
		L_pid->SetSetpoint(finding_speed);
		R_pid->SetSetpoint(finding_speed);
		break;
	case backward:
		L_pid->SetSetpoint(-finding_speed);
		R_pid->SetSetpoint(-finding_speed);
		break;
	case rotation:
		L_pid->SetSetpoint(rotate_speed);
		R_pid->SetSetpoint(-rotate_speed);
		break;
	case chase:
		diff = Dir_pid->output(target_x, ir_target->center.first);
		sprintf(data, "I:%d,%d\n", diff, 0);
		bt->SendStr(data);
		L_pid->SetSetpoint(chasing_speed - diff);
		R_pid->SetSetpoint(chasing_speed + diff);
		break;
	case stop:
		L_pid->SetSetpoint(0);
		R_pid->SetSetpoint(0);
		sprintf(data, "I:%d,%d\n", 0, 0);
		bt->SendStr(data);
		break;
	case out:
		L_pid->SetSetpoint(L_out_speed);
		R_pid->SetSetpoint(R_out_speed);
		break;
	case avoid:
		p = o_target.target->center;
		diff = target_slope2 * p.second + target_intercept2;
		diff = avoid_pid->output(diff, p.first);
		L_pid->SetSetpoint(chasing_speed - diff);
		R_pid->SetSetpoint(chasing_speed + diff);
		break;
	case approach:
		p = ir_target2.target->center;
		diff = target_slope2 * p.second + target_intercept2;
		diff = avoid_pid->output(diff, p.first);
		L_pid->SetSetpoint(chasing_speed - diff);
		R_pid->SetSetpoint(chasing_speed + diff);
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

#endif
