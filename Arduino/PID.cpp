# include "PID.h"

PID::PID (double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection) {
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    isAuto = false;

    PID::SetOutputLimits (0, 255);

    SampleTime = 100;

    PID::SetControllerDirection (controllerDirection);
    PID::SetTunings (Kp, Ki, Kd, POn);

    lastTime = millis () - SampleTime;
}

PID::PID (double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
    :PID::PID (Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {
}

bool PID::Compute () {
    /*  Compute () *************************************
     *  This function should be called every time "void loop ()" is executed. the function will decide whether a
     *  new PID output need to be computed. returns true when the output is computed, false when nothing has been done.
    ****************************************************/
    if (!isAuto) return false;
    unsigned long now = millis ();
    unsigned long timeChange = (now - lastTime);
    if (timeChange >= SampleTime) {
        // Compute all the working error variables
        double input = *myInput;
        double error = *mySetpoint - input;
        double dInput = (input - lastInput);
        outputSum += (ki * error);

        if (error >= 0) controllerDirection = DIRECT;
        else controllerDirection = REVERSE;

        // Add Proportional on Measurement, if P_ON_M is specified.
        if (!pOnE) outputSum -= kp * dInput;

        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin) outputSum = outMin;

        // Add Proportional on Error, if P_ON_E is specified.
        double output;
        if (pOnE) output = kp * error;
        else output = 0;

        // Compute the rest of PID output:
        output += outputSum - kd * dInput;

        if (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;
        *myOutput = output;

        // Remember some variables for next time
        lastInput = input;
        lastTime = now;

        return true;
    }
    else return false;
}

void PID::SetTunings (double Kp, double Ki, double Kd, int POn) {
    /*  SetTunings (...) *************************************
     *  This function allows the controller's dynamic performance to be adjusted.
     *  While it's automatically called by the constructor, tunings can also adjusted during operation
    ****************************************************/
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double) SampleTime) / 1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

   if (controllerDirection == REVERSE) {
    kp = (0 - kp);
    ki = (0 - ki);
    kp = (0 - kp);
   }
}

void PID::SetTunings (double Kp, double Ki, double Kd) {
    /*  SetTunings (...) *************************************
     *  Set Tunings using the last-remembered POn setting
    ****************************************************/
    SetTunings (Kp, Ki, Kd, pOn);
}

void PID::SetSampleTime (int NewSampleTime) {
    /*  SetSampleTime (...) *************************************
     *  Sets the period, in Milliseconds, at which the calculation is performed.
    ****************************************************/
    if (NewSampleTime > 0) {
        double ratio = (double) NewSampleTime / (double) SampleTime;

        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long) NewSampleTime;
    }
}

void PID::SetOutputLimits (double Min, double Max) {
    /*  SetOutputLimits (...) *************************************
     *  Clamp output to 0-125.
    ****************************************************/
   if (Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if (isAuto) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;

        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin) outputSum = outMin; 
   }
}

void PID::SetMode (int Mode) {
    /*  SetMode (...) *************************************
     *  Allows the controller Mode to be set to manual (0) or Automatic (1)
    ****************************************************/
    
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !isAuto) {
        PID::Initialize ();
    }
    isAuto = newAuto;
}

void PID::Initialize () {
    /*  Initialize () *************************************
     *  Ensures a bumpless transfer from manual to automatic mode or vice-versa
    ****************************************************/
   outputSum = *myOutput;
   lastInput = *myInput;
   if (outputSum > outMax) outputSum = outMax;
   else if (outputSum < outMin) outputSum = outMin;
}

void PID::SetControllerDirection (int Direction) {
    /*  SetControllerDirection (...) *************************************
     *  The PID will either be connected to a DIRECT acting process (+Output lead to +Input)
     *  or a REVERSE acting process (+Output leads to -Input). we need to know which one, because otherwise we may increase the
     *  output when we should be decreasing. This is called from the constructor.
    ****************************************************/
   if (isAuto && Direction != controllerDirection) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
   }
   controllerDirection = Direction;
}

double PID::GetKp () { return dispKp; }
double PID::GetKi () { return dispKi; }
double PID::GetKd () { return dispKd; }
int PID::GetMode () { return isAuto ? AUTOMATIC : MANUAL; }
int PID::GetDirection () { return controllerDirection; }