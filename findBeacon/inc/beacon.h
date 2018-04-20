/*
 * beacon.h
 *
 *  Created on: Apr 16, 2018
 *      Author: Sheldon
 */

#ifndef INC_BEACON_H_
#define INC_BEACON_H_

class Beacon {
public:
	uint16_t left_x;
	uint16_t right_x;
	uint16_t upper_y;
	uint16_t lower_y;
	uint16_t count;

	Beacon(uint16_t x, uint16_t y) :
			left_x(x), right_x(x), upper_y(y), lower_y(y), count(0) {
	}
	Beacon() :
			left_x(0), right_x(0), upper_y(0), lower_y(0), count(0) {
	}
	void init(uint16_t x, uint16_t y) {
		left_x = x;
		right_x = x;
		upper_y = y;
		lower_y = y;
		count = 0;
	}
};

#endif /* INC_BEACON_H_ */
