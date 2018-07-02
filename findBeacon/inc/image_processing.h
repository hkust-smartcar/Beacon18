/*
 * image_processing.h
 *
 *  Created on: May 3, 2018
 *      Author: Sheldon
 */

#ifndef INC_IMAGE_PROCESSING_H_
#define INC_IMAGE_PROCESSING_H_

#include <list>
#include "var.h"
#include "assert.h"

//////////////algo parm///////////////////
uint8_t x_range = 5;
uint8_t y_range = 35;
const uint16_t min_size = 50;
const uint8_t error = 30;
const uint16_t critical_density = 30;
const uint8_t max_beacon = 10;
const uint8_t frame = 10;
const uint8_t min_frame = 3;
const uint8_t near_dist = 40;
const uint32_t timeout = 50;
uint8_t frame_count = 0;
std::list<Record> center_record;
Beacon beacons[max_beacon];
uint8_t beacon_count = 0;

int16_t getX(uint16_t m_pos, int8_t m_bit_pos) {
	int16_t x = (m_pos + 1) * 8 % width - m_bit_pos - 1;
	if (x < 0)
		x = width - m_bit_pos - 1;
	return x;
}

int16_t getY(uint16_t m_pos) {
	int16_t y = m_pos * 8 / width;
	return y;
}

int8_t skip(uint16_t m_pos, int8_t m_bit_pos) {
	int16_t x = getX(m_pos, m_bit_pos);
	int16_t y = getY(m_pos);
	for (uint8_t i = 0; i < beacon_count; i++) {
		if (x < beacons[i].right_x + error && y < beacons[i].lower_y + error
				&& y > beacons[i].upper_y - error
				&& x > beacons[i].left_x - error) {
			return i;
		}
	}
	return -1;
}

// 1 = upper left, 2 = upper right , 3 = lower left, 4 = lower right
void sub_scan(uint16_t m_pos, int8_t m_bit_pos, int dir) {
	Beacon *current = beacons + beacon_count;
	switch (dir) {
	case 2:
		if (--m_bit_pos < 0) {
			m_bit_pos = 7;
			++m_pos;
		}
	case 1:
		if (m_pos - width / 8 < 0)
			return;
		else
			m_pos -= width / 8;
		break;
	case 4:
		if (--m_bit_pos < 0) {
			m_bit_pos = 7;
			++m_pos;
		}
	}

	uint8_t x_count = 0;
	uint8_t y_count = 0;
	uint8_t i = 0;
	bool all_black = true;
	int16_t x = 0;
	int16_t y = getY(m_pos);
	int16_t temp_pos = 0;

	while (x_count < x_range) {
		while (y_count < y_range) {
			temp_pos = dir > 2 ? m_pos + width * i / 8 : m_pos - width * i / 8;
			if (temp_pos >= numOfPixel || temp_pos < 0)
				break;
			if (!GET_BIT(buf[temp_pos], m_bit_pos)) {
				int16_t temp_y = dir > 2 ? y + i : y - i;
				x = getX(temp_pos, m_bit_pos);
				all_black = false;
				current->count++;
				if (dir % 2 != 0) { //scan left
					if (x < current->left_x)
						current->left_x = x;
				} else if (x > current->right_x) //scan right
					current->right_x = x;
				if (dir > 2) { //scan lower
					if (temp_y > current->lower_y)
						current->lower_y = temp_y;
				} else if (temp_y < current->upper_y) //scan upper
					current->upper_y = temp_y;
				//	y_count = 0;
			} else
				y_count++;
			i++;
		}

		if (all_black)
			x_count++;
		//		else
		//			x_count = 0;
		if (dir % 2 != 0) { //scan left
			if (++m_bit_pos > 7) {
				m_bit_pos = 0;
				--m_pos;
			}
		} else if (--m_bit_pos < 0) { //scan right
			m_bit_pos = 7;
			++m_pos;
		}
		y_count = 0;
		i = 0;
		all_black = true;
		if (m_pos * 8 / width != y)
			break;
	}
}

inline void check_target() {
	if (beacons[beacon_count].count > min_size
			&& beacons[beacon_count].density > critical_density)
		beacon_count++;
}

inline void scan(uint16_t m_pos, int8_t m_bit_pos, int mode) {

	int16_t x = getX(m_pos, m_bit_pos);
	int16_t y = getY(m_pos);
	beacons[beacon_count].init(x, y);

	//scan lower left
	sub_scan(m_pos, m_bit_pos, 3);
	//scan lower right
	sub_scan(m_pos, m_bit_pos, 4);

	if (mode == 1) {
		sub_scan(m_pos, m_bit_pos, 1);
		sub_scan(m_pos, m_bit_pos, 2);
	}

	beacons[beacon_count].calc();
//	beacon_count++;
	check_target();
}

inline bool check_near(const std::pair<uint16_t, uint16_t> b1,
		const std::pair<uint16_t, uint16_t> b2) {
	return abs(b1.first - b2.first) < near_dist
			&& abs(b1.second - b2.second) < near_dist;
}

inline void process() {
	buf = cam->LockBuffer();
	ir_target = NULL;
	beacon_count = 0;
	//////check for beacon with the last recorded pos/////////
	if (seen) {
		x_range = 30;
		y_range = 5;
		uint16_t temp_pos = (width * last_beacon.second) / 8
				+ last_beacon.first / 8;
		uint16_t temp_bit_pos = 8 - (last_beacon.first % 8 + 1);
		scan(temp_pos, temp_bit_pos, 1);
		if (beacon_count == 1)
			ir_target = beacons;
		cam->UnlockBuffer();
		return;
	}

	bool zero = true;
	uint16_t pos = 0;
	int8_t bit_pos = 9;
	x_range = 10;
	y_range = 20;
	for (uint16_t y = 0; y < height; y += 2) {
		for (uint16_t x = zero ? 0 : 1; x < width; x += 2) {
			bit_pos -= 2;
			if (bit_pos < 0) {
				bit_pos = zero ? 7 : 6;
				++pos;
			}
			if (!GET_BIT(buf[pos], bit_pos)) {
				int8_t i = skip(pos, bit_pos);
				if (i != -1) {
					uint16_t target_pos = beacons[i].right_x + error;
					if ((zero && target_pos % 2 != 0)
							|| (!zero && target_pos % 2 == 0))
						target_pos++;
					x = target_pos >= width ? width - 1 : target_pos;
					pos = (width * y) / 8 + x / 8;
					bit_pos = 8 - (x % 8 + 1);
					continue;
				}
				scan(pos, bit_pos, 0);
				if (beacon_count == max_beacon
						|| System::Time() - tick > timeout) {
					cam->UnlockBuffer();
					return;
				}
			}
		}
		zero = !zero;
		pos += width / 8;
	}
	if (beacon_count) { //have possible beacon but not met requirement
		if (frame_count == 0 || frame_count == frame) {
			frame_count = 0;
			center_record.clear();
		}
		++frame_count;
		for (int i = 0; i < beacon_count; i++) {
			bool add = false;
			for (auto it = center_record.begin(); it != center_record.end();
					++it)
				if (check_near(it->record.center, beacons[i].center)) {
					add = true;
					if (++it->count > min_frame) {
						ir_target = &it->record;
						frame_count = 0;
						cam->UnlockBuffer();
						return;
					}
				}
			if (!add)
				center_record.push_back(Record(beacons[i]));
		}
	}
	cam->UnlockBuffer();
}
#endif /* INC_IMAGE_PROCESSING_H_ */
