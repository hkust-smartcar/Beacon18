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
#endif /* INC_HELPER_H_ */
