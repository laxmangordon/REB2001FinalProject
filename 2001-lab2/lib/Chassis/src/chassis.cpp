#include "chassis.h"
#include "Romi32U4MotorTemplate.h"

Romi32U4EncodedMotor<LEFT_XOR, LEFT_B, PWM_L, DIR_L, OCR_L> leftMotor("L");
Romi32U4EncodedMotor<RIGHT_XOR, RIGHT_B, PWM_R, DIR_R, OCR_R> rightMotor("R");

/**
 * Because it's declared static, we initialize Chassis::loopFlag here.
 */
uint8_t Chassis::loopFlag = 0;
/**
 * For taking snapshots and raising the flag.
 */
void Chassis::Timer4OverflowISRHandler(void) 
{
    loopFlag++;

    leftMotor.speed = leftMotor.CalcEncoderDelta();
    rightMotor.speed = rightMotor.CalcEncoderDelta();
}
/**
 * ISR for timing. On Timer4 overflow, we take a 'snapshot' of the encoder counts 
 * and raise a flag to let the program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
   Chassis::Timer4OverflowISRHandler();
}
/**
 * Sets up a hardware timer on Timer4 to manage motor control on a precise schedule.
 * 
 * We set the timer to set an interrupt flag on overflow, which is handled
 * by ISR(TIMER4_OVF_vect) below.
 */
void Chassis::InitializeMotorControlTimer(void)
{
    Serial.println("InitTimer");
    // Disable interupts while we mess with the Timer4 registers
    cli(); 
  
    // Set up Timer4
    TCCR4A = 0x00; // Disable output to pins
    TCCR4B = 0x0A; // Sets the prescaler -- see pp. 167-8 in datasheet
    TCCR4C = 0x00; // Disables output to pins (but see below for buzzer)
    TCCR4D = 0x00; // Normal mode: count up and roll-over

    /**
     * Calculate TOP based on prescaler and loop duration. Note that loop is in integer ms --
     * there may be some rounding. Multiples of 4 ms will be exact.
     */
    uint16_t top = ((CONTROL_LOOP_PERIOD_MS * 16000ul) >> 9) - 1; // divides by 512

    /**
     * Here we do a little trick to allow full 10-bit register access. 
     * We have 2 _bits_ in TC4H that we can use to add capacity to TOP.
     * 
     * Note that the maximum period is limited by TOP = 0x3FF. If you want
     * a longer period, you'll need to adjust the pre-scaler.
     * 
     * There is no minumum period, but precision breaks down with low values, 
     * unless you adjust the pre-scaler, but the encoder resolution is limited,
     * so you only want to go so fast.
    */
    uint8_t highbits = top / 256;
    uint8_t lowbits = top - highbits;
    TC4H = highbits; OCR4C = lowbits;

    // Enable overflow interrupt
    TIMSK4 = 0x04; 

    /**
     * Uncommenting the following lines will pipe the timer signal to pin 6, 
     * which controls the buzzer. The pin will toggle at the loop rate, which
     * allows you to check that the timing is correct. It will also make a lot
     * of noise, so do so sparingly.
     */
    // TCCR4C = 0x04
    // pinMode(6, OUTPUT);

    // Re-enable interrupts
    sei(); 

    Serial.println("/InitTimer");
}

void Chassis::InititalizeChassis(void)
{
    InitializeMotorControlTimer();
    InitializeMotors();
}

/**
 * The main Chassis loop.
 */
bool Chassis::ChassisLoop(Twist& velocity)
{
    bool retVal = false;

    if (loopFlag)
    {
        // Throttle "missed update" warnings (human readable)
        static unsigned long lastWarnMs = 0;
        if (loopFlag > 1 && (millis() - lastWarnMs) > 2000)
        {
            lastWarnMs = millis();
            Serial.print("Missed updates: ");
            Serial.println(loopFlag - 1);
        }

        // IMPORTANT: this prints every tick if __LOOP_DEBUG__ is enabled.
        // Either disable __LOOP_DEBUG__ in platformio.ini, or throttle it like this:
#ifdef __LOOP_DEBUG__
        static unsigned long lastLoopPrintMs = 0;
        if (millis() - lastLoopPrintMs > 500)
        {
            lastLoopPrintMs = millis();
            Serial.print("tick @ ");
            Serial.println(millis());
        }
#endif

        // Update the wheel velocity so it gets back to Robot.
        velocity = CalcOdomFromWheelMotion();

        loopFlag = 0;
        retVal = true;
    }

    return retVal;
}
/**
 * Some motor methods.
 */
void Chassis::InitializeMotors(void)
{
    Romi32U4MotorBase::InitializePWMTimerAndInterrupts();

    leftMotor.InitializeMotor();
    rightMotor.InitializeMotor();
}

void Chassis::SetMotorEfforts(int16_t left, int16_t right) 
{
    leftMotor.SetMotorEffortDirect(left); 
    rightMotor.SetMotorEffortDirect(right);
}
int16_t Chassis::GetLeftEncoderTotal(void)
{
    return leftMotor.GetEncoderTotal();
}

int16_t Chassis::GetRightEncoderTotal(void)
{
    return rightMotor.GetEncoderTotal();
}


Twist Chassis::CalcOdomFromWheelMotion(void)
{
    // Control loop dt in seconds
    const float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;

    // Encoder deltas (ticks) captured each control loop in the ISR
    const float dTicksL = static_cast<float>(leftMotor.speed);
    const float dTicksR = static_cast<float>(rightMotor.speed);

    // Convert ticks -> cm travelled per loop
    const float dSL_cm = dTicksL / LEFT_TICKS_PER_CM;
    const float dSR_cm = dTicksR / RIGHT_TICKS_PER_CM;

    // Convert to wheel linear velocities in cm/s
    const float vL = dSL_cm / dt;
    const float vR = dSR_cm / dt;

    // Differential drive: forward velocity and yaw rate
    Twist velocity;
    velocity.u = 0.5f * (vR + vL);                  // cm/s forward in robot frame
    velocity.v = 0.0f;                              // always 0 in robot frame (per lab)
    velocity.omega = (vR - vL) / (2.0f * ROBOT_RADIUS); // rad/s if ROBOT_RADIUS in cm

#ifdef __NAV_DEBUG__
    TeleplotPrint("u", velocity.u);
    TeleplotPrint("omega", velocity.omega);
#endif

    return velocity;
}