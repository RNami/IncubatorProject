# ifndef PID_h
# define PID_h

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

class PID {

    public:

        # define AUTOMATIC 1
        # define MANUAL 0
        # define DIRECT 0
        # define REVERSE 1
        # define P_ON_M 0
        # define P_ON_E 1

        // Primary Functions ******************

        PID (double*, double*, double*,             // * Constructor: Input, Output and Setpoint.
             double, double, double, int);
        
        PID (double*, double*, double*,             // * Constructor: Input, Output and Setpoint. (overload for specifying proportional mode)
             double, double, double, int, int);

        void SetMode (int Mode);                    // * Sets PID Controller either Manual (0) or Auto (1)

        bool Compute ();                            // * Performs the PID Calculation. Should be called every loop () cycles.

        void SetOutputLimits (double, double);      // * Clamps the output to a specific range. 0-255 by default, but could change.

        // ******************


        // Secondary Functions ******************

        void SetTunings (double, double, double);   // * Function tp set the tunings, even in the runtime. (Could be used for Adaptive control)

        void SetTunings (double, double, double, int); // * Overload for specifying proportional mode

        void SetControllerDirection (int); // * Sets the Direction, or "Action" of the controller.
                                            // DIRECT means the output will increase when error is positive.
                                            // REVERSE means the opposite.
        
        void SetSampleTime (int);          // * Sets the frequency, in Milliseconds, with which the PID calculation is performed. default is 100

        // ******************


        // Display Functions ******************

        double GetKp ();                   // * Functions to query the PID for internal values.
        double GetKi ();
        double GetKd ();
        int GetMode ();
        int GetDirection ();

        // ******************



    private:

        void Initialize ();

        double dispKp;                    // * Tuning Paramters in user-entered format for display purposes.  
        double dispKi;
        double dispKd;

        double kp;                       // * (P)roportional Tuning Parameter
        double ki;                       // * (I)ntegral Tuning Parameter
        double kd;                       // * (D)erivative Tuning Parameter

        int controllerDirection;
        int pOn;

        double *myInput;                // * Pointers to the Input, Output, and Setpoint Variables
        double *myOutput;               // * This creates a hard link between the variables and the PID,
        double *mySetpoint;             // * freeing the user from having to constantly tell us what these values are.

        unsigned long lastTime;
        double outputSum, lastInput;

        unsigned long SampleTime;
        double outMin, outMax;
        bool isAuto, pOnE;
};
# endif
