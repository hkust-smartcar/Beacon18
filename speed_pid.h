/*
 * speed_pid.h
 * Created on: June 8, 2018
 * Author: Wongky
 */

#ifndef SPEED_PID
#define SPEED_PID

#include "libutil/pid_controller.h"
#include "libsc/timer.h"

template<typename InT_, typename OutT_>
class SpeedPid : public libutil::PidController<InT_, OutT_>
{
public:
	typedef InT_ InT;
	typedef OutT_ OutT;

	SpeedPid(const InT setpoint, const float kp, const float ki,
			const float kd);

	/**
	 * Set the upper bound of i, <= 0 means unlimited i
	 *
	 * @param value
	 */
	void SetILimit(const float value)
	{
		m_i_limit = value;
	}

	/**
	 * Set the upper bound of difference of error to ingore the error in i, <= 0 means unlimited diff
	 *
	 * @param value
	 */
	void SetEDiffIgnore(const float value)
	{
		m_e_diff_ignore = value;
	}

	/**
	 * Set the upper bound of whether add the new error, <= 0 means unlimited diff
	 *
	 * @param value
	 */
	void SetEDiffInclude(const float value)
	{
		m_e_diff_include = value;
	}

	inline void SetChnageSpeed(const bool value, InT_ newSpeed)
	{
		if(newSpeed!=this->GetSetpoint())
		{
			changeSpeed = value;
			this->SetSetpoint(newSpeed);
		}

	}

	void Reset()
	{
		ResetTime();
	}

	void ResetTime()
	{
		m_prev_time = libsc::System::Time();
	}

protected:
	void OnCalc(const InT error) override;
	OutT GetControlOut() override;

private:
	float m_i_limit;
	float m_e_diff_ignore;
	float m_e_diff_include;

	int32_t errorSum;
	bool changeSpeed;

	InT m_prev_error[2];
	InT m_prev_ierror;
	OutT m_prev_output;
	libsc::Timer::TimerInt m_prev_time;
};

#endif

#include "speed_pid.tcc"
