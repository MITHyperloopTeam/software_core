#ifndef BRAKE_SYSTEM_H
#define BRAKE_SYSTEM_H

#include "PodComponent.h"
#include "Vec3d.h"
#include "yaml-cpp/yaml.h"
#include <lcm/lcm.h>
#include "lcmgen_c/mithl_pin_sim_t.h"

class BrakeSystem : public PodComponent {
  public:
    BrakeSystem(YAML::Node config, lcm_t * lcm, double t_0);
    void reset(double t_0);
    void update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque);
    void output(double t);
    void pinSimHandler(const mithl_pin_sim_t * msg);
  private:
    lcm_t * lcm_;
    mithl_pin_sim_t pin_states_;


    // physical sim params

    //  The brakes consist of two brake calipers, constrained together with
    //  the primary brake cylinder, both free to slide laterally.
    double shift_amt_; // lateral shift of entire brake assembly relative to origin
    double cylinder_open_amt_; // length of cylinder constraining brake pads. positive is more open
    double cylinder_open_amt_vel_; // velocity of cylinder. 
    double system_pressure_; // pressure in cylinder side. Increases rapidly as fluid has nowhere to go.

    // The state variables are related as:
    // d shift_amt_ / dt = <unused>
    // d cylinder_open_amt_ / dt = cylinder_open_amt_vel_
    // d cylinder_open_amt_vel_ / dt = system_pressure_ * CYLINDER_AREA  - CONSTANT_SPRING_FORCE - DAMPING * cylinder_open_amt_vel_
    // d system_pressure_ / dt = - PRESSURE_VELOCITY_SCALE * cylinder_open_amt_vel_ + PUMP_SCALING * pump_cmd
    //                                   - min( ((1.0 - on_off_cmd) * ON_OFF_FLOW + on_off_cmd * ON_OFF_LEAKAGE)
    //                                          ((1.0 - prop_cmd)) * PROP_FLOW + prop_cmd * PROP_LEAKAGE) )

    const double Cylinder_Area = 0.1;
    const double Constant_Spring_Force = 1.0;
    const double Damping = 0.1;
    const double Pressure_Velocity_Scale = 10000.0;
    const double Pump_Scaling = 1000.0; 
    const double On_Off_Leakage = 1.0;
    const double On_Off_Flow = 10000.0;
    const double Prop_Leakage = 500.0;
    const double Prop_Flow = 5000.0;

    
    // mechanical constraints on cylinder openness
    const double Cylinder_Open_Max = 4.0; // TODO(gizatt): this is obviously incorrect
    const double Cylinder_Open_Min = 0.5;

    const double Cylinder_Pot_Noise_Amplitude = 0.1;
    const double Midpoint_Distance = 0.0;

    // controller interface params, copied from BrakeActuatorController (danger if those change!!!)
     const int Pin_PWREN = 40;  //Enable all Brake Interface PCB power
    //Pump motor
    const int Pin_PumpMotorPWM = 7;      // Speed PWM to the Pump Motor
    const int Pin_PumpMotorEnable = 41; //Enable Pump Motor
    const unsigned int Pin_Encoder = 30;

    //ON-OFF Valve
    const int Pin_ONOFFValve = 34; //Enable ON/OFF Valve
    //Propotional Valve, missing hardware pin, need to revise on second version main PCB
    const int Pin_ValvePWM = 8;      // Speed PWM to the propotional valve
    const int Pin_ValveEnable = 33;  // Enable / disable prop valve 

    //Low speed motor Enable/Disable
    const int Pin_LSMotorEnable = 37; //Enable LS motor
    const int Pin_LSMotorDisable = 36; //Disable LS motor (after turn Pin 51 LOW, has to turn this HIGH)
    //Low speed actuator Enable/Disable
    const int Pin_LSActuatorEnable = 39; //Enable LS actuator
    const int Pin_LSActuatorDisable = 38; //Disable LS actuator (after turn Pin 50 LOW, has to turn this HIGH)
    //Definitions of the sensor signals
    const int Pin_LD1 = 9; //Linear pot 1 analog pin to A9
    const int Pin_LD2 = 2; //Linear pot 2 analog pin to A2
    // Encoder info
    const int Encoder_Pulse_Per_Rev = 12;
    //Pressure
    const int Pin_Pressure = 1; //Reading the pressure 

    // velocity to force lookup table
    // todo move to yaml
    double velocity_reference[3] = {0, 5.0, 10.0};
    double drag_reference[3] = {0.0, 8979.936, 10938.368}; // N at corresponding velocity
};

#endif