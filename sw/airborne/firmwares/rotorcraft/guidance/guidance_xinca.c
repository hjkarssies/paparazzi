/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 *
 * Based on the papers:
 * Cascaded Incremental Nonlinear Dynamic Inversion Control for MAV Disturbance Rejection
 * https://www.researchgate.net/publication/312907985_Cascaded_Incremental_Nonlinear_Dynamic_Inversion_Control_for_MAV_Disturbance_Rejection
 *
 * Gust Disturbance Alleviation with Incremental Nonlinear Dynamic Inversion
 * https://www.researchgate.net/publication/309212603_Gust_Disturbance_Alleviation_with_Incremental_Nonlinear_Dynamic_Inversion
 */
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_xinca.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/radio_control.h"
#include "subsystems/actuators.h"
#include "state.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/guidance/wls/wls_alloc_guidance.h"

// The acceleration reference is calculated with these gains. If you use GPS,
// they are probably limited by the update rate of your GPS. The default
// values are tuned for 4 Hz GPS updates. If you have high speed position updates, the
// gains can be higher, depending on the speed of the inner loop.
#ifdef GUIDANCE_INDI_POS_GAIN
float guidance_indi_pos_gain = GUIDANCE_INDI_POS_GAIN;
#else
float guidance_indi_pos_gain = 0.5;
#endif

#ifdef GUIDANCE_INDI_SPEED_GAIN
float guidance_indi_speed_gain = GUIDANCE_INDI_SPEED_GAIN;
#else
float guidance_indi_speed_gain = 1.8;
#endif

#ifndef GUIDANCE_INDI_ACCEL_SP_ID
#define GUIDANCE_INDI_ACCEL_SP_ID ABI_BROADCAST
#endif
abi_event accel_sp_ev;
static void accel_sp_cb(uint8_t sender_id, uint8_t flag, struct FloatVect3 *accel_sp);
struct FloatVect3 indi_accel_sp = {0.0, 0.0, 0.0};
bool indi_accel_sp_set_2d = false;
bool indi_accel_sp_set_3d = false;

struct FloatVect3 sp_accel = {0.0, 0.0, 0.0};

static void guidance_indi_filter_actuators(void);

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

float act_z = 0;
float act_x = 0;

Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass act_z_filt;
Butterworth2LowPass act_x_filt;

float control_increment[XINCA_NUM_ACT]; // [dtheta, dphi, dact_z, dact_x]

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

float time_of_accel_sp_2d = 0.0;
float time_of_accel_sp_3d = 0.0;

struct FloatEulers guidance_euler_cmd;
float act_z_in;
float act_x_in;

//XINCA specific parameters
int num_iter_xinca = 0;
int num_cycle_xinca = 1; // Cycle count (resets every nth cycle)
#ifdef GUIDANCE_XINCA_NTH_CYCLE
int run_nth_cycle_xinca = GUIDANCE_XINCA_NTH_CYCLE; // Run every nth cycle
#else
int run_nth_cycle_xinca = 1; // Run every nth cycle
#endif

float G[XINCA_OUTPUTS][XINCA_NUM_ACT];
float *B[XINCA_OUTPUTS];
float v_xinca[XINCA_OUTPUTS];
float du_xinca[XINCA_NUM_ACT];
float du_min_xinca[XINCA_NUM_ACT];
float du_max_xinca[XINCA_NUM_ACT];
float du_pref_xinca[XINCA_NUM_ACT];
float du_prev_xinca[XINCA_NUM_ACT];

float c_l_alpha = GUIDANCE_XINCA_C_L_ALPHA;
float c_z_thrust = GUIDANCE_XINCA_C_Z_THRUST;
float c_x_tail_rotor = GUIDANCE_XINCA_C_X_TAIL_ROTOR;
float mass = GUIDANCE_XINCA_MASS;
float wing_surface = GUIDANCE_XINCA_WING_SURFACE;

#ifdef GUIDANCE_XINCA_RHO
float rho = GUIDANCE_XINCA_RHO;
#else
float rho = 1.225;
#endif

#ifdef GUIDANCE_XINCA_GRAVITY
float gravity = GUIDANCE_XINCA_GRAVITY;
#else
float gravity = 9.81;
#endif

#ifdef GUIDANCE_XINCA_ACT_Z_TAU
float act_z_tau = GUIDANCE_XINCA_ACT_Z_TAU;
#else
float act_z_tau = 30;
#endif
float act_z_dyn;

#ifdef GUIDANCE_XINCA_ACT_X_TAU
float act_x_tau = GUIDANCE_XINCA_ACT_X_TAU;
#else
float act_x_tau = 60;
#endif
float act_x_dyn;

#ifdef GUIDANCE_XINCA_U_PREF
float u_pref[XINCA_NUM_ACT] = GUIDANCE_XINCA_U_PREF;
#else
float u_pref[XINCA_NUM_ACT] = {0, 0, 0, 0};
#endif

#ifdef GUIDANCE_XINCA_W_ACC
float W_acc[XINCA_OUTPUTS] = GUIDANCE_XINCA_W_ACC;
#else
float W_acc[XINCA_OUTPUTS] = {10, 10, 1};
#endif

#ifdef GUIDANCE_XINCA_W_ACT
float W_act[XINCA_NUM_ACT] = GUIDANCE_XINCA_W_ACT;
#else
float W_act[XINCA_NUM_ACT] = {10, 10, 100, 1};
#endif

#ifdef GUIDANCE_XINCA_GAMMA
float gamma_sq = GUIDANCE_XINCA_GAMMA;
#else
float gamma_sq = 10000;
#endif

#ifdef GUIDANCE_XINCA_MAX_ITER
float max_iter_xinca = GUIDANCE_XINCA_MAX_ITER;
#else
float max_iter_xinca = 10;
#endif

#ifdef GUIDANCE_XINCA_H_THRES
float h_thres = GUIDANCE_XINCA_H_THRES;
#else
float h_thres = 0.2;
#endif

static void guidance_indi_propagate_filters(struct FloatEulers *eulers);
static void guidance_xinca_calcG_yxz(struct FloatEulers *euler_yxz);

/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void)
{

  act_z_dyn = 1 - exp(-act_z_tau / (PERIODIC_FREQUENCY / run_nth_cycle_xinca));
  act_z_in = stabilization_cmd[COMMAND_THRUST];
  act_z = act_z_in;
  
  act_x_dyn = 1 - exp(-act_x_tau / (PERIODIC_FREQUENCY / run_nth_cycle_xinca));
  act_x_in = stabilization_cmd[COMMAND_PITCH];
  act_x = act_x_in;

  float_vect_zero(du_prev_xinca, XINCA_NUM_ACT);

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / (PERIODIC_FREQUENCY / run_nth_cycle_xinca);
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&act_z_filt, tau, sample_time, act_z_in);
  init_butterworth_2_low_pass(&act_x_filt, tau, sample_time, act_x_in);
}

/**
 * @param heading_sp the desired heading [rad]
 *
 * main indi guidance function
 */
void guidance_indi_run(float *heading_sp)
{

  // Only compute the XINCA command once every run_nth_cycle cycles
  if (num_cycle_xinca >= run_nth_cycle_xinca) {
    num_cycle_xinca = 1;
  } else {
    num_cycle_xinca += 1;
    return;
  }

  struct FloatEulers eulers_yxz;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  float_eulers_of_quat_yxz(&eulers_yxz, statequat);

  // Filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters(&eulers_yxz);

  // Linear controller to find the acceleration setpoint from position and velocity
  float pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x) - stateGetPositionNed_f()->x;
  float pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y) - stateGetPositionNed_f()->y;
  float pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z);

  float speed_sp_x = pos_x_err * guidance_indi_pos_gain;
  float speed_sp_y = pos_y_err * guidance_indi_pos_gain;
  float speed_sp_z = pos_z_err * guidance_indi_pos_gain;

  // If the acceleration setpoint is set over ABI message
  if (indi_accel_sp_set_2d) {
    sp_accel.x = indi_accel_sp.x;
    sp_accel.y = indi_accel_sp.y;
    // In 2D the vertical motion is derived from the flight plan
    sp_accel.z = (speed_sp_z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
    float dt = get_sys_time_float() - time_of_accel_sp_2d;
    // If the input command is not updated after a timeout, switch back to flight plan control
    if (dt > 0.5) {
      indi_accel_sp_set_2d = false;
    }
  } else if (indi_accel_sp_set_3d) {
    sp_accel.x = indi_accel_sp.x;
    sp_accel.y = indi_accel_sp.y;
    sp_accel.z = indi_accel_sp.z;
    float dt = get_sys_time_float() - time_of_accel_sp_3d;
    // If the input command is not updated after a timeout, switch back to flight plan control
    if (dt > 0.5) {
      indi_accel_sp_set_3d = false;
    }
  } else {
    sp_accel.x = (speed_sp_x - stateGetSpeedNed_f()->x) * guidance_indi_speed_gain;
    sp_accel.y = (speed_sp_y - stateGetSpeedNed_f()->y) * guidance_indi_speed_gain;
    sp_accel.z = (speed_sp_z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
  }

#if GUIDANCE_INDI_RC_DEBUG
#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
  //for rc control horizontal, rotate from body axes to NED
  float psi = stateGetNedToBodyEulers_f()->psi;
  float rc_x = -(radio_control.values[RADIO_PITCH] / 9600.0) * 8.0;
  float rc_y = (radio_control.values[RADIO_ROLL] / 9600.0) * 8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  //for rc vertical control
  sp_accel.z = -(radio_control.values[RADIO_THROTTLE] - 4500) * 8.0 / 9600.0;
#endif

  // Calculate matrix of partial derivatives
  guidance_xinca_calcG_yxz(&eulers_yxz);

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0],sp_accel.y - filt_accel_ned[1].o[0], sp_accel.z - filt_accel_ned[2].o[0]};

  // Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  // Filter actuator estimation
  guidance_indi_filter_actuators();

  // Minimum increment in pitch angle, roll angle, thrust and tail rotor input
  du_min_xinca[0] = -guidance_indi_max_bank - pitch_filt.o[0];
  du_min_xinca[1] = -guidance_indi_max_bank - roll_filt.o[0];
  du_min_xinca[2] = (MAX_PPRZ - act_z_filt.o[0]) / c_z_thrust / XINCA_G_SCALING;
  du_min_xinca[3] = -act_x_filt.o[0] / c_x_tail_rotor / XINCA_G_SCALING;

  // Maximum increment in pitch angle, roll angle, thrust and tail rotor input
  du_max_xinca[0] = guidance_indi_max_bank - pitch_filt.o[0];
  du_max_xinca[1] = guidance_indi_max_bank - roll_filt.o[0];
  du_max_xinca[2] = -act_z_filt.o[0] / c_z_thrust / XINCA_G_SCALING;
  du_max_xinca[3] = (MAX_PPRZ - act_x_filt.o[0]) / c_x_tail_rotor / XINCA_G_SCALING;

  // Preferred increment in pitch angle, roll angle, thrust and tail rotor input
  du_pref_xinca[0] = u_pref[0] - pitch_filt.o[0];
  du_pref_xinca[1] = u_pref[1] - roll_filt.o[0];
  du_pref_xinca[2] = u_pref[2] - act_z_filt.o[0];
  du_pref_xinca[3] = u_pref[3] - act_x_filt.o[0];

  // Calculate virtual input
  v_xinca[0] = a_diff.x;
  v_xinca[1] = a_diff.y;
  v_xinca[2] = a_diff.z;

  // WLS Control Allocator
  wls_alloc_guidance(du_xinca, v_xinca, du_min_xinca, du_max_xinca,
      B, du_prev_xinca, 0, W_acc, W_act, du_pref_xinca, 10000, 10);
  float_vect_copy(du_prev_xinca, du_xinca, XINCA_NUM_ACT);

  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, du_xinca[2]);

  // Add increment in angles
  guidance_euler_cmd.theta = pitch_filt.o[0] + du_xinca[0];
  guidance_euler_cmd.phi = roll_filt.o[0] + du_xinca[1];
  guidance_euler_cmd.psi = *heading_sp;

  //Add increment in thrust and tail rotor input
  act_z_in = act_z_filt.o[0] + du_xinca[2] * c_z_thrust * XINCA_G_SCALING;
  Bound(act_z_in, 0, 9600);

  act_x_in = act_x_filt.o[0] + du_xinca[3] * c_x_tail_rotor * XINCA_G_SCALING;
  Bound(act_x_in, 0, 9600);

#if GUIDANCE_INDI_RC_DEBUG
  if (radio_control.values[RADIO_THROTTLE] < 300) {
    act_z_in = 0;
  }
#endif

  //Overwrite the thrust command from guidance_v
  stabilization_cmd[COMMAND_THRUST] = act_z_in;

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, -guidance_indi_max_bank, guidance_indi_max_bank);

  //set the quat setpoint with the calculated roll and pitch
  struct FloatQuat q_sp;
  float_quat_of_eulers_yxz(&q_sp, &guidance_euler_cmd);
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);

  //Commit tail rotor command
  if (-stateGetPositionNed_f()->z >= h_thres) {
    actuators_pprz[7] = act_x_in;
  } else {
    actuators_pprz[7] = -MAX_PPRZ;
  }
  
}

/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_actuators(void)
{
  // Actuator dynamics
  act_z = act_z + act_z_dyn * (act_z_in - act_z);
  act_x = act_x + act_x_dyn * (act_x_in - act_x);

  // Same filter as for the acceleration
  update_butterworth_2_low_pass(&act_z_filt, act_z);
  update_butterworth_2_low_pass(&act_x_filt, act_x);
}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 */
void guidance_indi_propagate_filters(struct FloatEulers *eulers)
{
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, eulers->phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers->theta);
}

/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the pitch, roll and thrust.
 * w.r.t. the NED accelerations for YXZ eulers
 * ddx = G*[dtheta,dphi,dT]
 */
void guidance_xinca_calcG_yxz(struct FloatEulers *euler_yxz)
{

  // Get rotation matrix from NED to body coordinates
  struct FloatRMat *ned_to_body_rmat = stateGetNedToBodyRMat_f();

  // Get vertical body acceleration
  struct FloatVect3 a_ned = {filt_accel_ned[0].o[0], filt_accel_ned[1].o[0], filt_accel_ned[2].o[0]};
  struct FloatVect3 a_body;
  float_rmat_vmult(&a_body, ned_to_body_rmat, &a_ned);

  // Get body velocity velocity (ideally airspeed velocity)
  struct NedCoor_f *ned_speed_f = stateGetSpeedNed_f();
  struct FloatVect3 speed_ned = {ned_speed_f->x, ned_speed_f->y, ned_speed_f->z};
  struct FloatVect3 speed_body;
  float_rmat_vmult(&speed_body, ned_to_body_rmat, &speed_ned);

  // Upward estimate minus gravity is an estimate of the thrust force
  float T = a_body.z - gravity;

  // Get current attitude angles
  float sphi = sinf(euler_yxz->phi);
  float cphi = cosf(euler_yxz->phi);
  float stheta = sinf(euler_yxz->theta);
  float ctheta = cosf(euler_yxz->theta);

  // Calculate matrix components
  G[0][0] = ctheta * cphi * T;
  G[1][0] = 0;
  G[2][0] = -stheta * cphi * T;// + c_l_alpha * 0.5 * rho * speed_body.x * speed_body.x * wing_surface / mass;
  G[0][1] = -stheta * sphi * T;
  G[1][1] = -cphi * T;
  G[2][1] = -ctheta * sphi * T;
  G[0][2] = stheta * cphi;
  G[1][2] = -sphi;
  G[2][2] = ctheta * cphi;

  // Only use tail rotor above threshold height
  if (-stateGetPositionNed_f()->z >= h_thres) {
    G[0][3] = ctheta;
    G[1][3] = 0;
    G[2][3] = -stheta;
  } else {
    G[0][3] = 0;
    G[1][3] = 0;
    G[2][3] = 0;
  }

  for (int i = 0; i < XINCA_OUTPUTS; i++) {
    B[i] = G[i];
  }

}

/**
 * ABI callback that obtains the acceleration setpoint from telemetry
 * flag: 0 -> 2D, 1 -> 3D
 */
static void accel_sp_cb(uint8_t sender_id __attribute__((unused)), uint8_t flag, struct FloatVect3 *accel_sp)
{
  if (flag == 0) {
    indi_accel_sp.x = accel_sp->x;
    indi_accel_sp.y = accel_sp->y;
    indi_accel_sp_set_2d = true;
    time_of_accel_sp_2d = get_sys_time_float();
  } else if (flag == 1) {
    indi_accel_sp.x = accel_sp->x;
    indi_accel_sp.y = accel_sp->y;
    indi_accel_sp.z = accel_sp->z;
    indi_accel_sp_set_3d = true;
    time_of_accel_sp_3d = get_sys_time_float();
  }
}

