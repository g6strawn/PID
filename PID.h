/******************************************************************************
PID.cpp - PID controller (aka autopilot)
Copyright 2017, Gary Strawn - Steal it, change it, use it however you'd like.

PID (Proportional, Integral, Derivative) is a control loop feedback mechanism.
A common example is a car's cruise control that adjusts the throttle smoothly.

See also:
 Intro: http://eli40.com/lander/02-debrief/#pid
 More: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 Arduino code: https://github.com/br3ttb/Arduino-PID-Library

A PID controller has three main variables:
  input  = process variable (PV) = measured value (ex: current speed)
  target = set point (SP)        = desired value (ex: cruise control setting)
  output = controlled value (OP) = computed value (ex: new throttle setting)

Assuming error=(target-input), dT=time interval in seconds, and Kp,Ki,Kd are 
tuned constants, then the computed output = (P+I-D) where:
  Proportional = Kp * error;                    //current error
  Integral     = Ki * Sum(error*dT);            //previous error (1/seconds)
  Derivative   = Kd * Sum((error-prevErr)/dT);  //future error = change rate (seconds)

A simple proportional controller (P only) would adjust the output by a set 
percentage (gain = Kp) of the measured error. However, too much gain will 
cause an overshoot (constantly swinging back and forth) while too little gain 
will cause an undershoot (never reaching the target).

The integral (I) remembers the past and adds more output when there is a 
large cumulative error (i.e. we're way off) and less when it's close. As the 
output approaches the desired target, the proportional value gets smaller and 
smaller. The integral helps push the output all the way to the target.

The derivative (D) predicts the future by adding in the rate of change. For 
example "We're coming in hot, start slowing down now." Or the opposite "This 
is taking forever, hurry up and get there."

The three tuning constants, or gains (Kp, Ki, Kd), are application specific.
Tuning Kp, Ki, and Kd is important but specific to every different usage.
  Reasonable starting values?  Kp=1, Ki=0.002, Kd=
  Gentle: Kp=1, Ki=0.05, Kd=0.25;  Aggressive: Kp=4, Ki=0.2, Kd=1
  Sebastian Thrun's steering example: Kp=0.2, Ki=0.004, Kd=3
Note on units: Ki = 1/seconds, Kd = seconds
Negative tuning constants will produce reverse output. This is rare but 
useful for things like a refrigerator where output = -(target-input)
******************************************************************************/
//Example usage of PID class:
#if 0  //Example usage:
PID pid(0.875, 0.5, 0.1);  //every usage will have unique constants
pid.Reset(output, input);  //using current values allows a smooth start

//call PID::Update() at even intervals
double prevTime = millis();  //start time in milliseconds
while(pid.IsRunning()) {
	//PID must be called at a constant time interval
	double currTime = millis();  //current time in milliseconds
	while((currTime - prevTime) > pid.Rate()) {  //is it time for another update?
		double output = pid.Update(input, target);
		prevTime += pid.Rate();
		//... do something with output
	}

	//Optional: adjust tuning constants to be more/less aggressive
	if(abs(target - input) < SOME_THRESHOLD)
		pid.SetTuning(gentleKp, gentleKi, gentleKd);  //we're close, be conservative
	else
		pid.SetTuning(powerKp, powerKi, powerKd);  //not even close, be aggressive
}
#endif

#ifndef PID_H
#define PID_H

//-----------------------------------------------------------------------------
//PID controller - This version assumes a constant sample rate (faster math).
class PID {
public:
	//these three are probably all you need
	PID(double kp, double ki, double kd, unsigned rate=100, double minOut=0, double maxOut=1);
	void Reset(double output, double input);  //perform a gentle restart; resume without jumping
	double Update(double input, double target);  //perform another PID calculation

	//the following adjustments are not usually necessary
	void SetTuning(double kp, double ki, double kd);  //adjust tuning constants (ex: gentle vs aggressive)
	void SetLimits(double min, double max);  //clamp output to specified range (ex: don't go beyond max throttle)
	void SetRate(unsigned rate);  //adjust sample rate, in milliseconds (shouldn't normally change)

	//return the same value that was set
	double Kp() const      { return m_kp; }
	double Ki() const      { return m_Ki / ((double)m_rate / 1000); }
	double Kd() const      { return m_Kd / ((double)m_rate * 1000); }

private:
	double Clamp(double d) const  { return (d < m_min) ? m_min : ((m_max < d) ? m_max : d); }

	//constants (changeable, but not every update)
	double m_kp, m_ki, m_kd;  //tuned constants (don't adjust directly, call SetTuning)
	double m_min, m_max;  //output limits
	unsigned m_rate;  //update rate (in milliseconds)

	//current state, changes every update
	double m_errorSum;   //integral
	double m_prevInput;  //derivative
};


//see also:  Twiddle() defined in PID.cpp

#endif
