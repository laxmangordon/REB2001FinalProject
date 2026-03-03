#pragma once

#include <Arduino.h>

#include "chassis-params.h"
#include "utils.h"

class Chassis
{
protected:

    /**
     * You can change the control loop period, but you should use multiples of 4 ms to 
     * avoid rounding errors.
     */
    const uint16_t CONTROL_LOOP_PERIOD_MS = 20;

    /**
     * loopFlag is used to tell the program when to update. It is set when Timer4
     * overflows (see InitializeMotorControlTimer). Some of the calculations are too
     * lengthy for an ISR, so we set a flag and use that to key the calculations.
     * 
     * Note that we use in integer so we can see if we've missed a loop. If loopFlag is
     * more than 1, then we missed a cycle.
     */
    static uint8_t loopFlag;

public:
    Chassis(void) {}
    void InititalizeChassis(void);

    /* Where the bulk of the work for the motors gets done. */
    bool ChassisLoop(Twist&);

    /* Needed for managing motors. */
    static void Timer4OverflowISRHandler(void);

public:
    Twist CalcOdomFromWheelMotion(void);
    int16_t GetLeftEncoderTotal(void);
    int16_t GetRightEncoderTotal(void);

    void SetMotorEfforts(int16_t, int16_t);

    void Stop(void) { SetMotorEfforts(0, 0);} 

protected:
    /**
     * Initialization and Setup routines
     */
    void InitializeMotorControlTimer(void);
    void InitializeMotors(void);
    
    void UpdateMotors(void);
};
