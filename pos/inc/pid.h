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
	int32_t max;
	int32_t min;
	//assume errorIgnore always positive
	int32_t errorIgnore;
	int32_t errorAcc;
	uint16_t numOfLower;
	int32_t errorAccept;

	PID() :
			kP(0), kI(0), kD(0), errorSumBound(0), max(0), min(0), preError(0), errorSum(0),
			last_time(0), target(0), errorIgnore(70),errorAcc(0),isAcc(false),numOfLower(0),errorAccept(0), isCount(false),changeTime(0) {
	}

	PID(const float& p, const float& i, const float& d, int m_max, int m_min) :
			kP(p), kI(i), kD(d), errorSumBound(0), max(m_max), min(m_min), preError(0),
			errorSum(0), last_time(0), target(0), errorIgnore(70),errorAcc(0),isAcc(false),numOfLower(0),errorAccept(0), isCount(false), changeTime(0){
	}
	void settarget(int32_t t) {
		if(target!=t)
		{
			target = t;
			errorAccept = abs(target * 0.4);
			changeTime = System::Time();
			isAcc = true;

		}

	}
	int32_t getTarget() {
		return target;
	}
	int32_t output(const int32_t& current) {
		uint32_t time_diff = System::Time() - last_time;
		int32_t error = target - current;

//		if(isAcc && System::Time() -changeTime>200)
//		{
//			isAcc = false;
//			errorAcc = 0;
//		}


		accuError(error);

		if(isCount)
		{
			if(target>0)
			{
				if(current<target && abs(error)>errorAccept)
				{
					numOfLower ++;
				}
				else
				{
					numOfLower=0;
				}
			}
			else if(target<0)
			{
				if(target<current && abs(error)>errorAccept)
				{
					numOfLower++;
				}
				else
				{
					numOfLower=0;
				}
			}
		}

		float ki = 0;
		if(isAcc)
		{
			ki = (errorSum + errorAcc) * kI;
		}
		else ki = errorSum * kI /** time_diff*/;
		float kd = (error - preError) * kD /*/ time_diff*/;
		float kp = error * kP;
		int32_t out = (int32_t) (kp + ki + kd);
		if(max != 0 && min != 0){
			if(out < min)
				out = min;
			else if(out > max)
				out = max;
		}
		preError = error;
		last_time = System::Time();
		//assume errorIgnore always positive
		if(!isAcc && abs(error)>errorIgnore)
		{
			delError(error);
		}
		return out;
	}

	void reset() {
		errorSum = 0;
		preError = 0;
		isAcc = false;
		isCount =false;
		errorAcc = 0;
		numOfLower= 0;
		last_time = System::Time();
		changeTime = 0;
	}

//	int32_t getErrorSum()
//	{
//		return errorSum;
//	}

	uint16_t getNumError()
	{
		return numOfLower;
	}

//	int32_t getErrorAcc()
//	{
//		return errorAcc;
//	}

	int32_t getIsAcc()
	{
		return isAcc;
	}

	void setIsAcc(bool f)
	{
		if(f==true && isAcc==false)
		{
			isAcc = true;
			errorAcc=0;
		}
		else if(f==false && isAcc==true)
		{
			isAcc = false;
			errorAcc=0;
		}

	}

	void setIsCount(bool f)
	{
		if(f==true && isCount==false)
		{
			isCount = true;
			numOfLower= 0;
		}
		else if(f==false && isCount==true)
		{
			isCount = false;
			numOfLower= 0;
		}
	}

	void resetAcc()
	{
		errorAcc=0;

	}


	bool getErrorFull()
	{
		if(abs(errorAcc) >= 5000)
		{
			return true;
		}
		return false;
	}

private:
	int32_t preError;
	int32_t errorSum;
	int32_t last_time;
	int32_t target;
	int32_t changeTime;
	bool isAcc;
	bool isCount;

	void accuError(const int32_t& e) {

		if(isAcc)
		{
			if ((errorAcc + e) > errorSumBound) {
				errorAcc = errorSumBound;
			} else if ((errorSum + e) < -errorSumBound) {
				errorAcc = -errorSumBound;
			} else {
				errorAcc += e;
			}
		}
		else
		{
			//assume errorSumBound always positive
			if ((errorSum + e) > errorSumBound) {
				errorSum = errorSumBound;
			} else if ((errorSum + e) < -errorSumBound) {
				errorSum = -errorSumBound;
			} else {
				errorSum += e;
			}
		}
	}

	void delError(const int32_t& e) {
		//assume errorSumBound always positive
		if ((errorSum - e) > errorSumBound) {
			errorSum = errorSumBound;
		} else if ((errorSum - e) < -errorSumBound) {
			errorSum = -errorSumBound;
		} else {
			errorSum -= e;
		}
	}
};


#endif /* PID_H */
