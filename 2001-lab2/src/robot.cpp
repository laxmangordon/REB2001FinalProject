// robot.cpp — FULL FIX (NO DONUTS) + Task 1/2 actuator scripts + Task 3 original drive array
//
// Key fix:
//   - Servo pulses are BLOCKING (delayMicroseconds). If you run them during navigation,
//     your chassis loop gets starved and the robot "donuts".
//   - So: while robotState == ROBOT_DRIVE_TO_POINT, we run ZERO servo pulses. Period.
//
// Buttons:
//   A -> Task 1 actuator script (timed Z/X/Claw)
//   B -> Task 2 actuator script (timed Z/X/Claw)
//   C -> Task 3 drive destination array (your original curved array)
//
// Manual mode:
//   Hold B + C to toggle manual mode. Motors OFF, prints pose, ignores scripts/driving.

#include "robot.h"
#include <Arduino.h>
#include <Romi32U4Buttons.h>
#include <math.h>

// ============================================================
// 1) PINS (edit ONLY if rewired)
// ============================================================
static const uint8_t PIN_Z_L  = 5;   // elevator left
static const uint8_t PIN_Z_R  = 12;  // elevator right
static const uint8_t PIN_X    = 11;  // x axis
static const uint8_t PIN_CLAW = 13;   // claw

// ============================================================
// 2) SERVO PULSE BASICS (FS90R continuous)
// ============================================================
static const uint16_t FRAME_US = 20000; // 50 Hz

static inline int16_t clampPulse(int32_t us)
{
    if (us < 900)  return 900;
    if (us > 2100) return 2100;
    return (int16_t)us;
}

// BLOCKING pulse for one 20 ms frame
static void pulseOneFrame(uint8_t pin, uint16_t pulse_us)
{
    pulse_us = (uint16_t)clampPulse(pulse_us);
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulse_us);
    digitalWrite(pin, LOW);
    delayMicroseconds(FRAME_US - pulse_us);
}

// ============================================================
// 3) TUNING (ONLY TOUCH THIS SECTION)
// ============================================================
//
// (A) STOP TRIMS  -------------------------------------------------
static int16_t STOP_Z_L  = 1450;  // tune creep
static int16_t STOP_Z_R  = 1470;  // tune creep
static int16_t STOP_X    = 1500;  // tune creep
static int16_t STOP_CLAW = 1500;  // tune creep
//
// (B) MIRROR SIGN -------------------------------------------------
static const int8_t Z_R_INV = -1;
//
// (C) SPEED DELTAS ------------------------------------------------
static int16_t Z_L_UP   = 170;
static int16_t Z_R_UP   = 195;
static int16_t Z_L_DOWN = 195;
static int16_t Z_R_DOWN = 170;

static int16_t X_IN  = 120;
static int16_t X_OUT = 120;

static int16_t CLAW_OPEN  = 120;
static int16_t CLAW_CLOSE = 120;

// ============================================================
// 4) BUTTONS
// ============================================================
static Romi32U4ButtonA button_a;
static Romi32U4ButtonB button_b;
static Romi32U4ButtonC button_c;

// ============================================================
// 5) ACTUATOR COMMANDS (-1/0/+1)
// ============================================================
static int8_t z_cmd = 0;     // +1 up, -1 down, 0 stop
static int8_t x_cmd = 0;     // +1 in, -1 out,  0 stop
static int8_t claw_cmd = 0;  // +1 open, -1 close, 0 stop

static uint32_t x_brake_until_ms = 0;
static uint32_t claw_brake_until_ms = 0;

static inline uint16_t pulseX()
{
    int32_t p = STOP_X;
    if (x_cmd > 0)      p = STOP_X + X_IN;   // in
    else if (x_cmd < 0) p = STOP_X - X_OUT;  // out
    return (uint16_t)clampPulse(p);
}

static inline uint16_t pulseClaw()
{
    int32_t p = STOP_CLAW;
    if (claw_cmd > 0)      p = STOP_CLAW + CLAW_OPEN;   // open
    else if (claw_cmd < 0) p = STOP_CLAW - CLAW_CLOSE;  // close
    return (uint16_t)clampPulse(p);
}

static inline void stopAllActuators()
{
    z_cmd = 0;
    x_cmd = 0;
    claw_cmd = 0;
    x_brake_until_ms = millis() + 250;
    claw_brake_until_ms = millis() + 250;
}

// Full apply (blocking): use ONLY when NOT driving
static inline void applyActuatorsBlocking()
{
    // --- Z (two servos) ---
    int32_t l = STOP_Z_L;
    int32_t r = STOP_Z_R;

    if (z_cmd > 0)
    {
        l = STOP_Z_L + Z_L_UP;
        r = STOP_Z_R + (Z_R_UP * Z_R_INV);
    }
    else if (z_cmd < 0)
    {
        l = STOP_Z_L - Z_L_DOWN;
        r = STOP_Z_R + ((-Z_R_DOWN) * Z_R_INV);
    }

    pulseOneFrame(PIN_Z_L, (uint16_t)clampPulse(l));
    pulseOneFrame(PIN_Z_R, (uint16_t)clampPulse(r));

    // --- X + claw (only if moving or in brake window) ---
    const uint32_t now = millis();

    if (x_cmd != 0 || (int32_t)(now - x_brake_until_ms) < 0)
        pulseOneFrame(PIN_X, pulseX());

    if (claw_cmd != 0 || (int32_t)(now - claw_brake_until_ms) < 0)
        pulseOneFrame(PIN_CLAW, pulseClaw());
}

// HARD DISABLE: ZERO pulses while driving.
// (Kept here so you don't accidentally bring it back later.)
static inline void refreshZStopOccasionally(bool /*driving*/)
{
    return;
}

// ============================================================
// 6) TASK 1/2 TIMED SCRIPTS (tune by editing ms ONLY)
// ============================================================
enum ScriptMode { SCRIPT_NONE=0, SCRIPT_TASK1, SCRIPT_TASK2 };
static ScriptMode script = SCRIPT_NONE;

struct Step { int8_t z, x, claw; uint16_t ms; };

// >>> TUNE HERE <<< Task 1 step times
static Step task1[] =
{
    {  +1, 0,  0, 5500 },
    {-1, 0,  0, 4500 },
};

// >>> TUNE HERE <<< Task 2 step times
static Step task2[] =
{

};

static uint8_t  step_i  = 0;
static uint32_t step_t0 = 0;

static void startScript(ScriptMode which)
{
    script = which;
    step_i = 0;
    step_t0 = millis();
    stopAllActuators();

    if (which == SCRIPT_TASK1) Serial.println("SCRIPT: Task1 START (A)");
    if (which == SCRIPT_TASK2) Serial.println("SCRIPT: Task2 START (B)");
}

static void runScript()
{
    if (script == SCRIPT_NONE) return;

    Step* arr = nullptr;
    uint8_t n = 0;

    if (script == SCRIPT_TASK1) { arr = task1; n = sizeof(task1)/sizeof(task1[0]); }
    if (script == SCRIPT_TASK2) { arr = task2; n = sizeof(task2)/sizeof(task2[0]); }

    if (!arr || n == 0)
    {
        script = SCRIPT_NONE;
        stopAllActuators();
        Serial.println("SCRIPT: ERROR (no steps)");
        return;
    }

    if (step_i >= n)
    {
        script = SCRIPT_NONE;
        stopAllActuators();
        Serial.println("SCRIPT: DONE");
        return;
    }

    z_cmd    = arr[step_i].z;
    x_cmd    = arr[step_i].x;
    claw_cmd = arr[step_i].claw;

    if (millis() - step_t0 >= arr[step_i].ms)
    {
        if (arr[step_i].x != 0)    x_brake_until_ms = millis() + 250;
        if (arr[step_i].claw != 0) claw_brake_until_ms = millis() + 250;

        step_i++;
        step_t0 = millis();
    }
}

// ============================================================
// 7) TASK 3 DESTINATION ARRAY (KNOWN GOOD WORKING CURVE)
// ============================================================
static const Pose k_destinations[] =
{
    // Up and left (curve outward)
    {   0.0f,  0.0f, 0.0f },
    {  -25.0f,  -15.0f, 0.0f },
    {-50.0f, 10.0f, 0.0f },

};

static const uint8_t k_num_destinations = sizeof(k_destinations) / sizeof(k_destinations[0]);

// ============================================================
// Robot lifecycle
// ============================================================
void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    pinMode(PIN_Z_L, OUTPUT);
    pinMode(PIN_Z_R, OUTPUT);
    pinMode(PIN_X, OUTPUT);
    pinMode(PIN_CLAW, OUTPUT);

    digitalWrite(PIN_Z_L, LOW);
    digitalWrite(PIN_Z_R, LOW);
    digitalWrite(PIN_X, LOW);
    digitalWrite(PIN_CLAW, LOW);

    Serial.println("READY.");
    Serial.println("A=Task1 script, B=Task2 script, C=Task3 drive array");
    Serial.println("NO DONUTS FIX: zero servo pulses during ROBOT_DRIVE_TO_POINT.");
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();
    robotState = ROBOT_IDLE;
    Serial.println("-> IDLE");
}

void Robot::RobotLoop(void)
{
    Twist velocity;
    if (!chassis.ChassisLoop(velocity)) return;

    // Always FK
    UpdatePose(velocity);

    static bool path_running = false;
    static uint8_t path_index = 0;

    static bool manual_mode = false;
    static bool last_bc_pressed = false;

    // Manual toggle: B + C held
    const bool b_pressed  = button_b.isPressed();
    const bool c_pressed  = button_c.isPressed();
    const bool bc_pressed = (b_pressed && c_pressed);

    if (bc_pressed && !last_bc_pressed)
    {
        manual_mode = !manual_mode;

        // cancel everything
        path_running = false;
        script = SCRIPT_NONE;
        stopAllActuators();

        robotState = ROBOT_IDLE;
        chassis.Stop();

        Serial.println(manual_mode ? "MANUAL MODE: ON" : "MANUAL MODE: OFF");
    }
    last_bc_pressed = bc_pressed;

    if (manual_mode)
    {
        chassis.SetMotorEfforts(0, 0);

        static uint32_t manual_last_print_ms = 0;
        if (millis() - manual_last_print_ms >= 100)
        {
            manual_last_print_ms = millis();
            Serial.print("x:"); Serial.print(currPose.x);
            Serial.print(" y:"); Serial.print(currPose.y);
            Serial.print(" th:"); Serial.println(currPose.theta);
        }

        // Hard stop actuators in manual
        stopAllActuators();
        applyActuatorsBlocking();
        return;
    }

    // ----------------------------
    // Button A -> Task 1 script
    // ----------------------------
    if (button_c.getSingleDebouncedPress())
    {
        path_running = false;
        robotState = ROBOT_IDLE;
        chassis.Stop();

        startScript(SCRIPT_TASK1);
    }

    // ----------------------------
    // Button B -> Task 2 script
    // ----------------------------
    if (button_b.getSingleDebouncedPress())
    {
        path_running = false;
        robotState = ROBOT_IDLE;
        chassis.Stop();

        startScript(SCRIPT_TASK2);
    }

    // ----------------------------
    // Button C -> Task 3 drive array
    // ----------------------------
    if (button_a.getSingleDebouncedPress())
    {
        // cancel scripts so nothing fights
        script = SCRIPT_NONE;
        stopAllActuators();

        // Reset pose at start (your original behavior)
        currPose.x = 0.0f;
        currPose.y = 0.0f;
        currPose.theta = 0.0f;

        path_running = true;
        path_index = 0;

        Serial.println("Path run: START");
        SetDestination(k_destinations[path_index]);

        // FORCE state so we don't accidentally zero motors in the "else" block
        robotState = ROBOT_DRIVE_TO_POINT;
    }

    // -----------------------------------------
    // Run scripts ONLY when NOT driving
    // -----------------------------------------
    const bool driving = (robotState == ROBOT_DRIVE_TO_POINT);

    if (!driving)
    {
        runScript();
        applyActuatorsBlocking(); // OK to block when not driving
    }
    else
    {
        // While driving: HARD OFF for actuators and ZERO servo pulses.
        script = SCRIPT_NONE;
        stopAllActuators();

        // DO NOT call applyActuatorsBlocking()
        // DO NOT call refreshZStopOccasionally()
    }

    // -----------------------------------------
    // Drive loop
    // -----------------------------------------
    if (robotState == ROBOT_DRIVE_TO_POINT)
    {
        DriveToPoint();

        if (CheckReachedDestination())
        {
            if (path_running)
            {
                path_index++;

                if (path_index < k_num_destinations)
                {
                    SetDestination(k_destinations[path_index]);
                    robotState = ROBOT_DRIVE_TO_POINT; // keep forced
                }
                else
                {
                    path_running = false;
                    Serial.println("Path run: DONE");
                    HandleDestination();
                }
            }
            else
            {
                HandleDestination();
            }
        }
    }
    else
    {
        // If not navigating, don't fight DriveToPoint
        chassis.SetMotorEfforts(0, 0);
    }

    // (Optional) tiny debug: state/pose at 5 Hz
    /*
    static uint32_t dbg_ms = 0;
    if (millis() - dbg_ms > 200)
    {
        dbg_ms = millis();
        Serial.print("state="); Serial.print((int)robotState);
        Serial.print(" x="); Serial.print(currPose.x);
        Serial.print(" y="); Serial.print(currPose.y);
        Serial.print(" th="); Serial.println(currPose.theta);
    }
    */
}
