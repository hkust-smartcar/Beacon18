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
	uint8_t density;
	uint16_t area;
	std::pair<uint16_t, uint16_t> center;

	Beacon(uint16_t x, uint16_t y) :
			left_x(x), right_x(x), upper_y(y), lower_y(y), count(0), density(0), area(
					0) {
	}
	Beacon(const Beacon& copy) {
		this->right_x = copy.right_x;
		this->left_x = copy.left_x;
		this->upper_y = copy.upper_y;
		this->lower_y = copy.lower_y;
		this->area = copy.area;
		this->density = copy.density;
		this->count = copy.count;
		this->center = copy.center;
	}

	Beacon(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2) :
			left_x(x1), right_x(x2), upper_y(y1), lower_y(y2), count(0), density(
					0), area(0) {
	}

	Beacon() :
			left_x(0), right_x(0), upper_y(0), lower_y(0), count(0), density(0), area(
					0){
	}

	void init(uint16_t x, uint16_t y) {
		left_x = x;
		right_x = x;
		upper_y = y;
		lower_y = y;
		count = 0;
		density = 0;
		area = 0;
	}

	void calc() {
		area = (lower_y - upper_y) * (right_x - left_x);
		float temp = count * 100 / area;
		density = temp;
		center = std::make_pair(left_x + (right_x - left_x) / 2,
				upper_y + (lower_y - upper_y) / 2);
	}

};

#endif /* INC_BEACON_H_ */
