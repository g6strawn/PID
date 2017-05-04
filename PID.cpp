/******************************************************************************
PID.cpp - PID controller (aka autopilot)
Copyright 2017, Gary Strawn - Steal it, change it, use it however you'd like.

PID (Proportional, Integral, Derivative) is a control loop feedback mechanism.
A common example is a car's cruise control that adjusts the throttle smoothly.

See documentation in .h file
******************************************************************************/
#include "PID.h"

//--------------------------------------------------------------
//Millis - current time in milliseconds;  Caution: wraps every 49.7 days
#if defined(ARDUINO)
#include "Arduino.h"
unsigned Millis() { return millis(); }  //capitalized to match non-library functions below

#elif defined(_WIN32)
unsigned Millis() { return GetTickCount(); }

#else  //assume Linux/Mac
#include <sys/time.h>
unsigned long Millis() {
#if 1 //gettimeofday
	timeval tm;
	gettimeofday(&tm, NULL);
#else //clock_gettime
	timespec tm;
	clock_gettime(CLOCK_MONOTONIC, &tm);
#endif
	return (tm.tv_sec * 1000) + (tm.tv_usec / 1000);
} //Millis
#endif


//-----------------------------------------------------------------------------
//PID::PID - constructor
PID::PID(double kp, double ki, double kd, unsigned rate, double minOut, double maxOut) : 
	m_rate(rate), m_min(minOut), m_max(maxOut)
{
	SetTuning(kp, ki, kd);
	Reset(0,0);  //Reset() should be called again with actual values
}


//-----------------------------------------------------------------------------
//PID::Reset - resume operation without jumping;  i.e. gentle restart
//Used when PID is switched from off (manual mode) to on (automatic mode)
void PID::Reset(double output, double input) {
	m_errorSum = Clamp(output);
	m_prevInput = input;
}


//-----------------------------------------------------------------------------
//PID::Update - return output = P+I+D
double PID::Update(double input, double target) {
	m_errorSum    = Clamp((m_ki * (target - input)) + m_errorSum);
	double output = Clamp((m_kp * (target - input)) + m_errorSum - (m_kd * (input - m_prevInput)));
	m_prevInput = input;
	return output;
}


//-----------------------------------------------------------------------------
//PID::SetTuning - adjust tuning constants (ex: gentle vs aggressive)
void PID::SetTuning(double kp, double ki, double kd) {
	double seconds = (double)m_rate / 1000.0;  //convert to seconds because ki = 1/sec and kd = sec
	m_kp = kp;
	m_ki = ki * seconds;
	m_kd = kd / seconds;
}


//-----------------------------------------------------------------------------
//PID::SetLimits - clamp output to specified range (ex: don't go beyond max throttle)
void PID::SetLimits(double min, double max) {
	m_min = min;
	m_max = max;
	m_errorSum = Clamp(m_errorSum);
}


//-----------------------------------------------------------------------------
//PID::SetRate - adjust sample rate, in milliseconds (shouldn't normally change)
void PID::SetRate(unsigned rate) {
	double ratio  = (double)rate / (double)m_rate;
	m_ki *= ratio;
	m_kd /= ratio;
	m_rate = rate;
}


//-----------------------------------------------------------------------------
//Twiddle - try to find good values for PID constants Kp, Ki, Kd
//  p    - array of parameters to be adjusted.    Start with {0,0,0}
//  dp   - array of adjustment amounts (deltas).  Start with {1,1,1}
//  len  - length of arrays. Usually 3 for PID
//  Test - pointer to test function that returns an error value for p
//  epsilon - test threshold; smaller values = more accuracy, longer runtime
//RETURNS: new values set in p
//Sebastian Thrun's explanation: https://www.youtube.com/watch?v=2uQ2BSzDvXs
#if 0 //EXAMPLE USAGE
double p[3]  = { 0,0,0 };  //parameters to be adjusted
double dp[3] = { 1,1,1 };  //delta-P = amount to adjust parameters
Twiddle(p, dp, 3, TwiddleTest);  //find values
#endif 0
extern double TwiddleTest(double* p, unsigned len);  //must be defined by user
extern void Twiddle(double* p, double* dp, unsigned len, void (*Test)(double*, unsigned), double epsilon=0.00001) {
//	assert(p && dp && len);  //assume valid parameters
	double bestErr = TwiddleTest(p, len);
	double e = epsilon + 1;
	while(e > epsilon) {
		for(unsigned i = 0;  i < len;  i++) {
			p[i] += dp[i];
			double err = TwiddleTest(p, len);
			if(err < bestErr) {
				//improvement, try a 10% larger value next time
				bestErr = err;
				dp[i] *= 1.1;
			} else {
				//did not improve, try a smaller value instead
				p[i] -= 2 * dp[i];
				err = TwiddleTest(p, len);
				if(err < bestErr) {
					//improvement, try another 10% next time
					bestErr = err;
					dp[i] *= 1.1;
				} else {
					//there was no improvement, try a smaller increment next time
					p[i] += dp[i];
					dp[i] *= 0.95;  //decrease step size
				}
			}
		}
		//calculate sum of dp[]
		e = 0;
		for(unsigned i = 0;  i < len;  i++)
			e += dp[i];
	}
}
