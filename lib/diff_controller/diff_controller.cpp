#include "diff_controller.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include <Arduino.h>
#include <cmath>

// pulses per wheel revolution (set to your encoder’s CPR)
static constexpr float ENCODER_PPR = 20.0f;
// max PWM magnitude
static constexpr int MAX_PWM       = 255;
// globals (defined extern in header)
PIDController leftPID;
PIDController rightPID;
float control_dt = 0.0f;
float wheel_base = 0.0f;
float wheel_radius = 0.0f;
unsigned char moving = 0;

void initDiffController(float dt,
                        float wheel_base_m,
                        float wheel_rad_m,
                        float kp_left, float ki_left, float kd_left,
                        float kp_right, float ki_right, float kd_right) {
  control_dt   = dt;
  wheel_base   = wheel_base_m;
  wheel_radius = wheel_rad_m;
  leftPID.Kp  = kp_left;
  leftPID.Ki  = ki_left * dt;
  leftPID.Kd  = kd_left / dt;
  leftPID.Ko  = 0;
  leftPID.TargetTicksPerFrame = 0;
  leftPID.PrevEnc             = 0;
  leftPID.ITerm               = 0;
  leftPID.output              = 0;
  leftPID.PrevInput           = 0;
  rightPID.Kp  = kp_right;
  rightPID.Ki  = ki_right * dt;
  rightPID.Kd  = kd_right / dt;
  rightPID.Ko  = 0;
  rightPID.TargetTicksPerFrame = 0;
  rightPID.PrevEnc             = 0;
  rightPID.ITerm               = 0;
  rightPID.output              = 0;
  rightPID.PrevInput           = 0;
  moving = 0;
}

void setDiffTarget(float v, float w) {
  // compute linear wheel speeds (m/s)
  float v_l = v - (w * wheel_base * 0.5f);
  float v_r = v + (w * wheel_base * 0.5f);
  // convert to rev/s
  float revs_l_per_s = v_l / (2.0f * M_PI * wheel_radius);
  float revs_r_per_s = v_r / (2.0f * M_PI * wheel_radius);
  // ticks per frame = rev/s * PPR * dt
  leftPID.TargetTicksPerFrame  = (long)(revs_l_per_s * ENCODER_PPR * control_dt);
  rightPID.TargetTicksPerFrame = (long)(revs_r_per_s * ENCODER_PPR * control_dt);
  moving = 1;
}

void resetPID() {
  resetEncoders();
  leftPID.PrevEnc   = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm     = 0;
  leftPID.output    = 0;
  rightPID.PrevEnc   = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm     = 0;
  rightPID.output    = 0;
  moving = 0;
}

void updatePID() {
  if (!moving) return;
  // read average encoder ticks per side
  long encL = (readEncoder(0) + readEncoder(3)) / 2;
  long encR = (readEncoder(1) + readEncoder(2)) / 2;
  // left side PID
  long deltaL = encL - leftPID.PrevEnc;
  long errL   = leftPID.TargetTicksPerFrame - deltaL;
  leftPID.ITerm += leftPID.Ki * errL;
  float dInputL = (encL - leftPID.PrevInput);
  float outL    = leftPID.Kp * errL + leftPID.ITerm - leftPID.Kd * dInputL;
  leftPID.output = constrain((int)outL, -MAX_PWM, MAX_PWM);
  leftPID.PrevEnc   = encL;
  leftPID.PrevInput = encL;
  // right side PID
  long deltaR = encR - rightPID.PrevEnc;
  long errR   = rightPID.TargetTicksPerFrame - deltaR;
  rightPID.ITerm += rightPID.Ki * errR;
  float dInputR = (encR - rightPID.PrevInput);
  float outR    = rightPID.Kp * errR + rightPID.ITerm - rightPID.Kd * dInputR;
  rightPID.output = constrain((int)outR, -MAX_PWM, MAX_PWM);
  rightPID.PrevEnc   = encR;
  rightPID.PrevInput = encR;
  // drive motors -> left motors are indices 0 & 3, right are 1 & 2
  int speeds[MOTOR_COUNT] = {
    (int)leftPID.output,
    (int)rightPID.output,
    (int)rightPID.output,
    (int)leftPID.output
  };
  setMotorSpeeds(speeds);
}