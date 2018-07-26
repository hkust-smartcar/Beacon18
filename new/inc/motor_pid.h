/*
 * pid.h
 *
 *  Created on: Mar 29, 2018
 *      Author: Sheldon
 */
#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <cstdint>
#include <libsc/system.h>
using namespace libsc;

class Motor_PID {
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
	float errorAcceptRate;
	uint16_t numOfLower;

	Motor_PID() :
			kP(0), kI(0), kD(0), errorSumBound(0), max(0), min(0), preError(0), errorSum(0),
			target(0), errorIgnore(70),errorAcceptRate(0.4),errorAcc(0),isAcc(false),numOfLower(0),errorAccept(0), isCount(false) {
	}

	Motor_PID(const float& p, const float& i, const float& d, int m_max, int m_min) :
			kP(p), kI(i), kD(d), errorSumBound(0), max(m_max), min(m_min), preError(0),
			errorSum(0),target(0), errorIgnore(70),errorAcceptRate(0.4),errorAcc(0),isAcc(false),numOfLower(0),errorAccept(0), isCount(false){
	}
	void settarget(const int32_t& t) {
		if(target!=t)
		{
			target = t;
			errorAccept = abs(target * 0.4);
			isAcc = true;
		}

	}
	int32_t getTarget() {
		return target;
	}
	int32_t output(const int32_t& current) {
		//uint32_t time_diff = System::Time() - last_time;
		int32_t error = target - current;

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
			if ((errorAcc + errorSum) > errorSumBound) {
				ki = errorSumBound * kI;
			} else if ((errorSum + errorSum) < -errorSumBound) {
				ki = -errorSumBound * kI;
			} else {
				ki = (errorSum + errorAcc) * kI;
			}
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
		//assume errorIgnore always positive
		if(!isAcc && abs(error)>errorIgnore)
		{
			delError(error);
		}
		return out;
	}

	void reset() {
		errorSum = 0; //5000
		preError = 0;
		isAcc = false;
		isCount =false;
		errorAcc = 0;
		numOfLower= 0;
	}


//acc
	int32_t getIsAcc()
	{
		return isAcc;
	}

	void setIsAcc(const bool& f)
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

	void addErrorAcc(const int32_t n)
	{
		if ((errorAcc + n) > errorSumBound) {
			errorAcc = errorSumBound;
		} else if ((errorSum + n) < -errorSumBound) {
			errorAcc = -errorSumBound;
		} else {
			errorAcc += n;
		}
	}

	void resetAcc()
	{
		isAcc = false;
		errorAcc=0;
	}


//num
	uint16_t getNumError()
	{
		return numOfLower;
	}

	void setIsCount(const bool& f)
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

private:
	int32_t preError;
	int32_t errorSum;
	int32_t target;
	int32_t errorAcc;
	int32_t errorAccept;
	bool isAcc;
	bool isCount;

	void accuError(const int32_t& e) {

		if(isAcc)
		{
			if ((errorAcc + e) > errorSumBound) {
				errorAcc = errorSumBound;
			} else if ((errorAcc + e) < -errorSumBound) {
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
