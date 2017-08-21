#include "PodComponent_BrakeSystem.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>

static void _pinSimHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_pin_sim_t * msg, void * user){
  ((BrakeSystem*)user)->pinSimHandler(msg);
}

static inline double randRange(double low, double high){
  return ((high - low)*((double)rand() / double(RAND_MAX)) + low);
}

BrakeSystem::BrakeSystem(YAML::Node config, lcm_t * lcm, double t_0)
                              : PodComponent(config, t_0), lcm_(lcm)
{
  printf("Initializing a brake system\n");
  mithl_pin_sim_t_subscribe(lcm_, "SIM_PINS_WRITE_LS", &_pinSimHandler, this);
  reset(t_0);
}

void BrakeSystem::reset(double t_0){
  last_update_time_ = t_0;
  system_pressure_ = 0.0;
  shift_amt_ = 0.0;
  cylinder_open_amt_ = 0.0;
  cylinder_open_amt_vel_ = 0.0;
}

void BrakeSystem::update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque)
{
  double dt = t - last_update_time_;
  last_update_time_ = t;

  // advance brake state based on analog inputs
  float pump_motor_state = 0.0; // off
  float pwm_valve_state = 0.0; // fully open
  float on_off_state = 0.0; // fully open

  if (pin_states_.digital[Pin_PWREN]){
    if (pin_states_.digital[Pin_PumpMotorEnable]){
      pump_motor_state = ((double)(pin_states_.analog[Pin_PumpMotorPWM]))/255.;
    }
    if (pin_states_.digital[Pin_ValveEnable]){
      pwm_valve_state = ((double)(pin_states_.analog[Pin_ValvePWM]))/255.;
    }
    if (pin_states_.digital[Pin_ONOFFValve]){
      on_off_state = 1.0; // fully closed
    }
  }


  // see header for the system update equations written in non-code form.
  double d_shift_amt_dt = 0.0;

  cylinder_open_amt_vel_ = 0.0;
  // Logistic curve fit to experimental data from prop valve given no motor flow rate
  cylinder_open_amt_vel_ +=  (1.0 - on_off_state) * -7.7803 / (1 + exp(42.0764*(pwm_valve_state - 0.7411)));
  // Linear curve fit to experimental data from motor given closed prop valve and on/off valve
  cylinder_open_amt_vel_ += 1.6196 * pump_motor_state;

  double d_cylinder_open_amt_dt = cylinder_open_amt_vel_;
  double d_cylinder_open_amt_vel_dt = 0.0;
  double d_system_pressure_dt = - Pressure_Velocity_Scale * cylinder_open_amt_vel_
                                + Pump_Scaling * pump_motor_state
                                - fmin( ((1.0 - on_off_state) * On_Off_Flow + on_off_state * On_Off_Leakage),
                                        ((1.0 - pwm_valve_state) * Prop_Flow + pwm_valve_state * Prop_Leakage));
  
  shift_amt_ += d_shift_amt_dt * dt;
  cylinder_open_amt_ += d_cylinder_open_amt_dt * dt;
  //cylinder_open_amt_vel_ += d_cylinder_open_amt_vel_dt * dt;

  system_pressure_ = 800.0 + randRange(-100.0, 100.0);

  if (cylinder_open_amt_ > Cylinder_Open_Max){
    system_pressure_ += 800;
    cylinder_open_amt_ = Cylinder_Open_Max;
  } else if (cylinder_open_amt_ < Cylinder_Open_Min){
    system_pressure_ -= 750;
    cylinder_open_amt_ = Cylinder_Open_Min;
  }

  // TODO(gizatt): implement pressure relief better
  system_pressure_ = fmin( fmax( system_pressure_, 0.0 ), 1500.0 );


  // compute braking force
  double braking_force = 0.0;
  if (fabs(xyz_dot.x()) <= velocity_reference[0]){
    braking_force = drag_reference[0];
  } else if (fabs(xyz_dot.x()) >= velocity_reference[2]){
    braking_force = drag_reference[2];
  } else {
    for (int i=0; i<2; i++){
      if (fabs(xyz_dot.x()) <= velocity_reference[i+1] && fabs(xyz_dot.x()) >= velocity_reference[i]){
        double mix_below = velocity_reference[i+1] - fabs(xyz_dot.x());
        double mix_above = fabs(xyz_dot.x()) - velocity_reference[i];
        braking_force = (mix_above * drag_reference[i+1] + mix_below * drag_reference[i]) / (mix_above + mix_below);
      }
    }
  }

  // make braking force oppose direction of motion
  if (xyz_dot.x() > 0){
    braking_force *= -1.0;
  }

  double openness_in_range = (cylinder_open_amt_ - Cylinder_Open_Min) / (Cylinder_Open_Max - Cylinder_Open_Min);
  braking_force *= (1.0 - openness_in_range);

  force = Vec3d(braking_force, 0.0, 0.0);
  torque = Vec3d(0.0);
}

void BrakeSystem::output(double t)
{
  mithl_pin_sim_t msg;

  msg.utime = t*1000*1000;

  for (unsigned int i=0; i<sizeof(msg.analog)/sizeof(msg.analog[0]); i++)
    msg.analog[i] = -1;
  for (unsigned int i=0; i<sizeof(msg.digital)/sizeof(msg.digital[0]); i++)
    msg.digital[i] = -1;
  
  // publish pressure TODO(gizatt): check this
  msg.analog[Pin_Pressure] = (int16_t)(((system_pressure_ * (3. - 0.6) / (3000)) + 0.6)*(1023. / 3.3));

  // publish pots TODO(gizatt): check this
  // distance first
  double ld1_dist = cylinder_open_amt_/2 + Midpoint_Distance + shift_amt_ + Cylinder_Pot_Noise_Amplitude * randRange(-1., 1.);
  double ld2_dist = cylinder_open_amt_/2 + Midpoint_Distance - shift_amt_ + Cylinder_Pot_Noise_Amplitude * randRange(-1., 1.);
  // convert to voltage
  double l1_voltage = ld1_dist / 40.54 / 0.03937 + 0.9;
  double l2_voltage = ld2_dist / 40.54 / 0.03937 + 0.9;
  msg.analog[Pin_LD1] = (int16_t) (l1_voltage * 1023.0 / 3.3);
  msg.analog[Pin_LD2] = (int16_t) (l2_voltage * 1023.0 / 3.3);

  mithl_pin_sim_t_publish((lcm_t *) lcm_, "SIM_PINS_READ_LS", &msg);
}

void BrakeSystem::pinSimHandler(const mithl_pin_sim_t * msg){
  memcpy(&pin_states_, msg, sizeof(mithl_pin_sim_t));
}
