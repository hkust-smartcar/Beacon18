/*
 * image_processing.h
 *
 *  Created on: May 3, 2018
 *      Author: Sheldon
 */

#ifndef INC_IMAGE_PROCESSING_H_
#define INC_IMAGE_PROCESSING_H_

#include <list>

extern const uint16_t width;
extern const uint16_t height;
extern const uint16_t numOfPixel;

//////////////algo parm///////////////////
const uint8_t x_range = 5;
const uint8_t y_range = 35;
const uint16_t min_size = 120;
const uint8_t error = 10;
const uint16_t critical_density = 65;
extern const uint8_t max_beacon;
extern Beacon* target;
extern Beacon last_beacon;
extern const uint8_t frame;
const uint8_t min_frame = 3;
const uint8_t near_dist = 40;
uint8_t frame_count = 0;
uint8_t count[20] = { };

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

bool skip(uint16_t m_pos, int8_t m_bit_pos, Beacon* beacons,
		uint8_t beacon_count) {
	int16_t x = getX(m_pos, m_bit_pos);
	int16_t y = getY(m_pos);
	for (uint8_t i = 0; i < beacon_count; i++) {
		if (x < beacons[i].right_x + error && y < beacons[i].lower_y + error
				&& y
						> (beacons[i].upper_y - error < 0 ?
								0 : beacons[i].upper_y - error)
				&& x
						> (beacons[i].left_x - error < 0 ?
								0 : beacons[i].left_x - error)) {
			return true;
		}
	}
	return false;
}

// 1 = upper left, 2 = upper right , 3 = lower left, 4 = lower right
void sub_scan(const Byte* buf, uint16_t m_pos, int8_t m_bit_pos,
		Beacon* current, int dir) {

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
				if (dir % 2 != 0) {					//scan left
					if (x < current->left_x)
						current->left_x = x;
				} else if (x > current->right_x) 	//scan right
					current->right_x = x;
				if (dir > 2) {						//scan lower
					if (temp_y > current->lower_y)
						current->lower_y = temp_y;
				} else if (temp_y < current->upper_y)	//scan upper
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
		if (dir % 2 != 0) {				//scan left
			if (++m_bit_pos > 7) {
				m_bit_pos = 0;
				--m_pos;
			}
		} else if (--m_bit_pos < 0) {		//scan right
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

bool check_target(Beacon *beacons, uint8_t beacon_count) {
	if (beacons[beacon_count].count > min_size)
		if (beacons[beacon_count].density > critical_density) {
			target = beacons + beacon_count;
			return true;
		}
	return false;
}

bool scan(const Byte* buf, uint16_t m_pos, int8_t m_bit_pos, Beacon* beacons,
		uint8_t &beacon_count, int mode) {

	int16_t x = getX(m_pos, m_bit_pos);
	int16_t y = getY(m_pos);
	beacons[beacon_count].init(x, y);

	//scan lower left
	sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 3);
	//scan lower right
	sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 4);

	if (mode == 1) {
		sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 1);
		sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 2);
	}

	beacons[beacon_count].calc();
	if (check_target(beacons, beacon_count))
		return true;
	else
		beacon_count++;
	return false;
}

bool check_near(Beacon b1, Beacon b2) {
	if (abs(b1.center.first - b2.center.first) < near_dist)
		if (abs(b1.center.second - b2.center.second) < near_dist)
			return true;
	return false;
}

bool process(const Byte* buf, Beacon* beacons, uint8_t &beacon_count, bool seen,
		std::list<Beacon>& center_record) {
	//////check for beacon with the last recorded pos/////////
	if (seen) {
		uint16_t temp_pos = (width * last_beacon.center.second) / 8
				+ last_beacon.center.first / 8;
		uint16_t temp_bit_pos = 8 - (last_beacon.center.first % 8 + 1);
		scan(buf, temp_pos, temp_bit_pos, beacons, beacon_count, 1);
		if (beacons[0].count > 50) {
			target = beacons;
			return true;
		} else
			beacon_count = 0;
	}
	uint16_t pos = 0;
	int8_t bit_pos = 8;
	for (uint16_t y = 0; y < height; y++) {
		for (uint16_t x = 0; x < width; x += 2) {
			bit_pos -= 2;
			if (bit_pos < 0) {
				bit_pos = 6;
				++pos;
			}
			if (!GET_BIT(buf[pos], bit_pos)) {
				if (beacon_count && skip(pos, bit_pos, beacons, beacon_count))
					continue;
				if (scan(buf, pos, bit_pos, beacons, beacon_count, 0))
					return true;
				if (beacon_count == max_beacon)
					return false;
			}
		}
	}
	if (beacon_count && target == NULL) { //have possible beacon but not met requirement
		if (frame_count == 0 || frame_count == frame) {
			frame_count = 0;
			center_record.clear();
		}
		++frame_count;
		for (int i = 0; i < beacon_count; i++) {
			bool add = false;
			for (auto it = center_record.begin(); it != center_record.end();
					++it) {
				if (check_near(*it, beacons[i])) {
					add = true;
					if (++count[std::distance(center_record.begin(), it)]
							> min_frame) {
						target = &(*it);
						frame_count = 0;
					}
					break;
				}
			}
			if (!add) {
				center_record.emplace_back(beacons[i]);
				count[center_record.size() - 1] = 1;
			}
		}

	}
	return false;
}
#endif /* INC_IMAGE_PROCESSING_H_ */
