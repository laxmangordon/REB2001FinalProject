#pragma once

#include <Arduino.h>
#include <FastGPIO.h>

// define the motor pins here
#define PWM_L 10
#define PWM_R 9
#define DIR_L 16
#define DIR_R 15

#define LEFT_XOR   8
#define LEFT_B     IO_E2
#define RIGHT_XOR  7
#define RIGHT_B    23

#define OCR_L 0x8A
#define OCR_R 0x88

class Romi32U4MotorBase
{
protected: 
    // For prettier printing
    const String name;

    /**
     * This is the speed of the motor, in "encoder counts / encoder interval".
     * The encoder interval is set in Robot::InitializeMotorControlTimer.
     */ 
    volatile int16_t speed = 0;

    // Maximum effort
    int16_t maxEffort = 420;

    // Keeps track of encoder changes
    volatile int16_t prevCount;
    volatile int16_t encCount;
    volatile int8_t lastA;
    volatile int8_t lastB;

    /**
     * Sets the duty cycle. Must be declared for specific motor.
     */
    virtual void SetEffort(int16_t effort) = 0;

    /** 
     * Used to set the motor _effort_ directly, which is mostly used for testing.
     * 
     * Calling this function will switch the control mode to DIRECT, if needed, meaning speed control is lost.
     */
    void SetMotorEffortDirect(int16_t effort)
    {
        SetEffort(effort);
    }

    /**
     * calcEncoderDelta() takes a 'snapshot of the encoders and 
     * stores the change since the last call, which has units of "encoder ticks/motor interval" 
     * 
     * We also use the function for zeroing the delta (for example, when switching control modes),
     * so interrupts are cleared when accessing encCount.
     */
    int16_t CalcEncoderDelta(void)
    {
        cli();
        int16_t currCount = encCount;
        sei();

        int16_t speed = currCount - prevCount;
        prevCount = currCount;

        return speed;
    }

    /**
     * GetEncoderTotal() returns the total encoder counts" 
     * 
     */
    int16_t GetEncoderTotal(void)
    {
        cli();
        int16_t currCount = encCount;
        sei();

        return currCount;
    }



    static void AttachInterrupts(void);

    /**
     * InitializeMotorPWMTimer() should be called near the beginning of the program.
     * It sets up Timer4 to run at 38 kHz, which is used to both drive the PWM signal for the motors
     * and (tangentially) allow for a 38 kHz signal on pin 11, which can be used, say, to drive an
     * IR LED at a common rate.
     * 
     * Timer 1 has the following configuration:
     *  prescaler of 1
     *  outputs enabled on channels A (pin 9), B (pin 10) and C (pin 11)
     *  fast PWM mode
     *  top of 420, which will be the max speed
     *  frequency is then: 16 MHz / [1 (prescaler) / (420 + 1)] = 38.005 kHz
     * */
    static void InitializePWMTimerAndInterrupts(void)
    {
        Serial.println("InitMotor()");

        noInterrupts(); //disable interrupts while we set Timer1 registers

        TCCR1A = 0xA2; //0b10100010; //Fast PWM + enable A and B; change to 0xAA to enable C on pin 11
        TCCR1B = 0x19; //0b00011001; //Fast PWM
        ICR1 = 420;    //runs at 38kHz; lowers speed for given effort by 5% from Pololu version

        //set all three outputs to zero
        OCR1A = 0;
        OCR1B = 0;
        OCR1C = 0; //can be used to create 38 kHz signal on pin 11; must enable output in TCCR1A above
        
        /**
         * Call a static function to set up the left and right motor interrupts
         */
        AttachInterrupts();

        interrupts(); //re-enable interrupts

        Serial.println("/InitMotor()");
    }

public:
    Romi32U4MotorBase(const String& nm) : name(nm) {}
};

template <uint8_t encXOR, uint8_t encB, uint8_t PWM, uint8_t DIR, uint8_t OCR> 
    class Romi32U4EncodedMotor : public Romi32U4MotorBase
{
protected:
    void InitializeMotor(void)
    {
        FastGPIO::Pin<PWM>::setOutputLow();
        FastGPIO::Pin<DIR>::setOutputLow();

        InitializeEncoder();
    }

    /**
     * SetEffort is used internally to set the motor effort without changing the control mode.
     */
    void SetEffort(int16_t effort)
    {
        bool reverse = 0;

        if (effort < 0)
        {
            effort = -effort; // Make speed a positive quantity.
            reverse = 1;      // Reverse the direction.
        }
        if (effort > maxEffort)
        {
            effort = maxEffort;
        }

        FastGPIO::Pin<DIR>::setOutput(reverse);

        /**
         * This line is a little esoteric, but it sets the duty cycle for the appropriate PWM pin.
         * Note that TOP for the PWM is 420, so the duty cycle will be (effort / 420) * 100 (in %).
         */
        _SFR_MEM16(OCR) = effort;
    }

    void InitializeEncoder(void)
    {    
        Serial.println("InitEnc()");

        // Set the pins as pulled-up inputs.
        FastGPIO::Pin<encXOR>::setInputPulledUp();
        FastGPIO::Pin<encB>::setInputPulledUp();

        // Initialize the variables so that the speed will start as 0
        lastB = FastGPIO::Pin<encB>::isInputHigh();
        lastA = FastGPIO::Pin<encXOR>::isInputHigh() ^ lastB;

        Serial.println("/InitEnc()");
    }

public:
    Romi32U4EncodedMotor(const String& name) : Romi32U4MotorBase(name) {}
    void ProcessEncoderTick(void) 
    {
        bool newB = FastGPIO::Pin<encB>::isInputHigh();
        bool newA = FastGPIO::Pin<encXOR>::isInputHigh() ^ newB;

        encCount += (lastA ^ newB) - (newA ^ lastB);

        lastA = newA;
        lastB = newB;
    }

    friend class Chassis; // Allow Chassis to call protected methods directly
};

extern Romi32U4EncodedMotor<LEFT_XOR, LEFT_B, PWM_L, DIR_L, OCR_L> leftMotor;
extern Romi32U4EncodedMotor<RIGHT_XOR, RIGHT_B, PWM_R, DIR_R, OCR_R> rightMotor;
