/*
 * pid.h
 *
 *  Created on: Mar 29, 2018
 *      Author: Sheldon
 */
#ifndef PID_H
#define PID_H

#include <cstdint>
#include <libsc/system.h>
using namespace libsc;

class PID {
public:
	float kP;
	float kI;
	float kD;

	//assume errorSumBound always positive
	int32_t errorSumBound;

	PID() :
			kP(0), kI(0), kD(0), errorSumBound(0), preError(0), errorSum(0) {
	}

	PID(const float& p, const float& i, const float& d) :
			kP(p), kI(i), kD(d), errorSumBound(0), preError(0), errorSum(0) {
	}

	int32_t output(const int32_t& target, const int32_t& current) {
		int32_t error = target - current;
		accuError(error);
		int32_t out = (int32_t)(
				error * kP + errorSum * kI + (error - preError) * kD);
		preError = error;
		return out;
	}

	void reset() {
		errorSum = 0;
		preError = 0;
		errorSum = 0;
	}

private:
	int32_t preError;
	int32_t errorSum;
	void accuError(const int32_t& e) {
		//assume errorSumBound always positive
		if ((errorSum + e) > errorSumBound) {
			errorSum = errorSumBound;
		} else if ((errorSum + e) < -errorSumBound) {
			errorSum = -errorSumBound;
		} else {
			errorSum += e;
		}
	}
};

#endif /* PID_H */
