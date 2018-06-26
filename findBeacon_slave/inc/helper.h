#ifndef INC_HELPER_H_
#define INC_HELPER_H_

#include "var.h"

void insert(Beacon t, ptr_mode m) {
	Beacon** ptr = NULL;
	switch (m) {
	case o:
		ptr = &o_target;
		break;
	case oRecord:
		ptr = &o_record;
		break;
	case irRecord:
		ptr = &ir_record;
		break;
	case ir_Target:
		ptr = &ir_target;
		break;
	case o_Target:
		ptr = &o_target;
		break;
	}
	if (*ptr != NULL) {
		delete *ptr;
		*ptr = NULL;
	}
	*ptr = new Beacon(t);
}
bool comm_listener(const Byte *data, const size_t size) {
    if(data[0] == 's') {
			run = true;
//			led0->Switch();
		}
		if(data[0] == 'S')
			run =false;
		return true;
}

void send_coord(uint8_t type) {
	BitConsts a;
	uint8_t buffer[5];
	buffer[0] = a.kSTART;
	Beacon *ptr = NULL;
	switch (type) {
	case PkgType::irTarget:
		buffer[1] = PkgType::irTarget;
		ptr = ir_target;
		break;
	case PkgType::oTarget:
		buffer[1] = PkgType::oTarget;
		ptr = o_target;
		break;
	}
	buffer[2] = (uint8_t) ptr->center.first;
	buffer[3] = ptr->center.second;
	buffer[4] = a.kEND;
	comm->SendBuffer(buffer, 5);
}

inline void reset_recrod() {
	if (ir_target != NULL) {
		insert(*ir_target, ptr_mode::irRecord);
		delete ir_target;
		ir_target = NULL;
	}

	if (o_target != NULL) {
		insert(*o_target, ptr_mode::oRecord);
		delete o_target;
		o_target = NULL;
	}
}

#endif /* INC_HELPER_H_ */
