// robot-nav.cpp
#include "robot.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// -------- Tunables --------
static const float k_dist_gain = 2.0f;
static const float k_head_gain = 6.0f;

static const float k_dist_cap_cm = 40.0f;
static const float k_u_max = 25.0f;
static const float k_omega_max = 4.0f;

static const float k_goal_tol_cm = 2.0f;

static const float k_effort_per_cm_s = 10.25f;
static const int16_t k_effort_max = 220;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float wrap_to_pi(float a)
{
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void Robot::UpdatePose(const Twist& twist)
{
    static uint32_t last_update_ms = 0;
    const uint32_t now_ms = millis();

    if (last_update_ms == 0)
    {
        last_update_ms = now_ms;
        return;
    }

    const float dt = (float)(now_ms - last_update_ms) / 1000.0f;
    last_update_ms = now_ms;

    const float u = twist.u;         // cm/s
    const float omega = twist.omega; // rad/s

    const float theta_mid = currPose.theta + 0.5f * omega * dt;

    currPose.x += u * cos(theta_mid) * dt;
    currPose.y += u * sin(theta_mid) * dt;
    currPose.theta += omega * dt;

    // Wrap theta to [-pi, pi]
    currPose.theta = wrap_to_pi(currPose.theta);
}

void Robot::SetDestination(const Pose& dest)
{
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    const float dx = destPose.x - currPose.x;
    const float dy = destPose.y - currPose.y;
    const float rho = sqrtf(dx * dx + dy * dy);
    return (rho <= k_goal_tol_cm);
}

void Robot::DriveToPoint(void)
{
    if (robotState != ROBOT_DRIVE_TO_POINT) return;

    const float dx = destPose.x - currPose.x;
    const float dy = destPose.y - currPose.y;

    float rho = sqrtf(dx * dx + dy * dy);
    rho = clampf(rho, 0.0f, k_dist_cap_cm);

    const float theta_goal = atan2f(dy, dx);
    const float alpha = wrap_to_pi(theta_goal - currPose.theta);

    float u = k_dist_gain * rho;
    float omega = k_head_gain * alpha;

    u = clampf(u, -k_u_max, k_u_max);
    omega = clampf(omega, -k_omega_max, k_omega_max);

    // Differential drive
    const float v_l = u - omega * ROBOT_RADIUS;
    const float v_r = u + omega * ROBOT_RADIUS;

    int16_t left_effort  = (int16_t)(k_effort_per_cm_s * v_l);
    int16_t right_effort = (int16_t)(k_effort_per_cm_s * v_r);

    if (left_effort > k_effort_max) left_effort = k_effort_max;
    if (left_effort < -k_effort_max) left_effort = -k_effort_max;
    if (right_effort > k_effort_max) right_effort = k_effort_max;
    if (right_effort < -k_effort_max) right_effort = -k_effort_max;

    chassis.SetMotorEfforts(left_effort, right_effort);
}

void Robot::HandleDestination(void)
{
    EnterIdleState();
}