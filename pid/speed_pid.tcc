/*
 * speed_pid.tcc
 * Created on: June 8, 2018
 * Author: Wongky
 */
 
template<typename InT_, typename OutT_>
SpeedPid<InT_, OutT_>::SpeedPid(
		const InT setpoint, const float kp, const float ki, const float kd)
		: libutil::PidController<InT_, OutT_>(setpoint, kp, ki, kd),
		  m_i_limit(0.0f),
		  m_e_diff_ignore(0.0f),
		  m_e_diff_include(0.0f),
		  errorSum(0),
		  changeSpeed(false),
		  m_prev_ierror(0),
		  m_prev_error{0, 0},
		  m_prev_output(0),
		  m_prev_time(libsc::System::Time())
{
this->SetSetpoint(0);
}

template<typename InT_, typename OutT_>
void SpeedPid<InT_, OutT_>::OnCalc(const InT error)
{
	using namespace libsc::k60;
	using namespace libsc;

	const Timer::TimerInt time = System::Time();
	
/////////////////////////
//	if(changeSpeed == true)
//	{
//		m_prev_time = System::Time();
//		
//	}
///////////////////////
	
	const float time_diff = Timer::TimeDiff(System::Time(), m_prev_time)/ 1000.0f;
	
	if(changeSpeed)
	{
		m_prev_time = System::Time();
		changeSpeed = false;
		
	}


	//kp* change of error//
	const float p = this->GetKp() * (error - m_prev_error[0]); 


	float i = 0;
	//i = this->GetKi() * (error + m_prev_ierror) * time_diff * 0.5f;
	//check if stable error
	if(m_e_diff_include > 0.0f)
	{
		if((error>=0 && error<= m_e_diff_include)
		|| (error<0 && error>= -m_e_diff_include))
		{
			i = this->GetKi() * (error + m_prev_ierror) * time_diff * 0.5f;
			//i = this->GetKi() * error * (time_diff);
			m_prev_time = time;
			m_prev_ierror = error;
		}
	}
	
	if (m_e_diff_ignore > 0.0f)
	{
		if((error>=0 && error>= m_e_diff_ignore)
		|| (error<0 && error<= -m_e_diff_ignore))
		{
			//i = this->GetKi() * (error + m_prev_ierror) * time_diff * 0.5f;
			m_prev_ierror = 0;		
		}
	}
	
	if (m_e_diff_include > 0.0f && m_e_diff_ignore > 0.0f)
	{
		if(error>=0 && error> m_e_diff_include && error<m_e_diff_ignore) //+ve error
		{
			i = this->GetKi() * error * (time_diff);
			m_prev_ierror = error;
			
		}
		else if (error<0 && error < -m_e_diff_include && error>-m_e_diff_ignore) //-ve error
		{
			i = this->GetKi() * error * (time_diff);
			m_prev_ierror = error;
		}		
	}
	
	
/*	
	if((error>=0 && error>20) || (error<0 && error<20))
	{
		i = this->GetKi() * (error) * (time_diff);
	}
	else 
	{i = this->GetKi() * (error + m_prev_error[0]) * time_diff * 0.5f;}
*/
	
	//avoid i sum too large
	if (m_i_limit > 0.0f)
	{
		i = libutil::Clamp<float>(-m_i_limit, i, m_i_limit);
	}
	
	
	//kd * (change of error - last change of error)//
	const float d = this->GetKd() * (error - 2 * m_prev_error[0] + m_prev_error[1]) / time_diff;

	std::swap(m_prev_error[0], m_prev_error[1]);
	m_prev_error[0] = error;
	//m_prev_time = time;
	//m_prev_ierror=iError*index;
	this->UpdatePid(p, i, d);
//	if(changeSpeed)
//	{
//		changeSpeed = false;
//	}
}

template<typename InT_, typename OutT_>
OutT_ SpeedPid<InT_, OutT_>::GetControlOut()
{
	return this->GetP() + this->GetI() + this->GetD();
}


 