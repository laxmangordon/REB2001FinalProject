// servo32u4.cpp - Library for controlling servos on the ATmega32U4's hardware PWM pins (5, 6, 12, and 13)
#include <servo32u4.h>

uint16_t Servo32U4Base::setMinMaxMicroseconds(uint16_t min, uint16_t max)
{
    // swap if in the wrong place
    if(min > max) {uint16_t temp = min; min = max; max = temp;}

    usMin = min;
    usMax = max;

    return usMax - usMin; //return the range, in case the user wants to do a sanity check
}

void Servo32U4Pin5::attach(void) 
{
    pinMode(5, OUTPUT); // set pin as OUTPUT

    cli();

    // clear then set the OCR3A bits (pin 5)
    TCCR3A = 0x82; //WGM
    TCCR3B = 0x1A; //WGM + CS = 8
    ICR3 = 39999; //20ms

    sei();

    isAttached = true;
}

void Servo32U4Pin5::detach(void)
{
    cli();

    // clear the OCR3A bits
    TCCR3A &= 0x7f; //cancel OCR3A
    sei();

    isAttached = false;
}

void Servo32U4Pin5::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 8, so 1 timer count = 0.5 us
    OCR3A = (microseconds << 1) - 1; // multiplies by 2
}

void Servo32U4Pin6::attach(void) 
{
    pinMode(6, OUTPUT); // set pin as OUTPUT

    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C |= 0x05;

    sei();

    isAttached = true;
}

void Servo32U4Pin6::detach(void) 
{
    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C &= ~0x05;

    sei();

    isAttached = false;
}


// Resolution is 64 us; not great, but shouldn't be too constraining
void Servo32U4Pin6::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 512, so 1 timer count = 32 us
    //but be sure to set TC4H first!
    TC4H = 0;
    OCR4D = (microseconds >> 5) - 1; // divides by 32
}

void Servo32U4Pin13::attach(void) 
{
    pinMode(13, OUTPUT); // set pin as OUTPUT

    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4A |= 0x82;

    sei();

    isAttached = true;
}

void Servo32U4Pin13::detach(void) 
{
    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    // If you're not using it for anything else, you can safely just set TCCR4A = 0,
    // but we'll cancel the bits we set above
    TCCR4A &= ~0x82;

    sei();

    isAttached = false;
}

// Resolution is 64 us; not great, but shouldn't be too constraining
void Servo32U4Pin13::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 512, so 1 timer count = 32 us
    //but be sure to set TC4H first!
    TC4H = 0;
    OCR4A = (microseconds >> 5) - 1; // divides by 32
}

void Servo32U4Pin12::attach(void) 
{
    pinMode(12, OUTPUT); // set pin as OUTPUT

    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C |= 0x05;

    //Invert the output on pin 12
    TCCR4B |= 0x80;

    sei();

    isAttached = true;
}

void Servo32U4Pin12::detach(void) 
{
    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C = 0x00;
    TCCR4B &= ~0x80;

    sei();

    isAttached = false;
}

// Resolution is 64 us; not great, but shouldn't be too constraining
void Servo32U4Pin12::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 512, so 1 timer count = 32 us
    //but be sure to set TC4H first!
    TC4H = 0;
    OCR4D = (microseconds >> 5) - 1; // divides by 32
}
