#!/usr/bin/env python
"""
Relays telemtry values from our system to SpaceX via telemetry spec (see docs)
"""
import socket
import struct
import time
import lcm
from lcm_utils import *
from mithl import net_health_t, flight_control_high_rate_t, state_t, bac_config_t, trigger_t
from mithl import spacex_telemetry_state_t
from mithl import state_estimator_particle_set

TARGET_IP = "192.168.0.1"
#TARGET_IP = "127.0.0.1"
TARGET_PORT = 3000
MITHL_ID = 27

STATUS_FAULT = 0     # triggers SpaceX run abort. If an estop is thrown while in 
                     # ARM, LAUNCH, or FLIGHT, a persistent FAULT flag is thrown
                     # until cleared through the interface or restarting this relay.
STATUS_IDLE = 1      # pod on but not ready to be pushed. 
STATUS_READY = 2     # Ready to be pushed. When in ARM, we publish this.
STATUS_PUSHING = 3   # Pod detects it is being pushed. When in LAUNCH, publish this.
STATUS_COAST = 4     # Pod detects it has separated from pusher vehicle. When in FLIGHT, publish this.
STATUS_BRAKING = 5   # Pod is applying its brakes. When in FLIGHT and brakes are less open than FULL, publish this.

MESSAGE_SEND_PERIOD = 0.0333

WATCHDOG = 2.0

fault_thrown = False
last_heard_estop = time.time()

state = None
stmsg = state_t()
last_heard_state = time.time() - WATCHDOG

particles = None
last_heard_state_estimate = time.time() - WATCHDOG

high_power_battery_info = None
last_heard_high_battery = time.time() - WATCHDOG

pod_temp = None
last_heard_pod_temp = time.time() - WATCHDOG

stripe_count = None
last_heard_fiducial = time.time() - WATCHDOG

brake_pos = None
last_heard_brake_state = time.time() - WATCHDOG

def handle_trigger_t(channel, data):
    global fault_thrown, last_heard_estop
    if (channel == "CLEAR_FAULT"):
        if state == stmsg.ESTOP:
            fault_thrown = False
        else:
            print "MUST BE IN ESTOP TO CLEAR FAULT"
    elif (channel == "_ESTOP" and (not state or state is not "ESTOP")):
        last_heard_estop = time.time()

def handle_state_t(channel, data):
    global state, last_heard_state
    try:
        msg = state_t.decode(data)
        state = msg.currentState
        last_heard_state = time.time()
    except Exception as e:
        print "Exception in state handler ", channel, ":", e


def handle_state_estimate(channel, data):
    global particles, last_heard_state_estimate
    msg = state_estimator_particle_set.decode(data)
    
    new_particles = []
    for i in range(msg.n_particles):
        if msg.particles[i].id >= 0 and msg.particles[i].weight > 0.:
            new_particles.append([msg.particles[i].mu,
                              msg.particles[i].Sigma,
                              msg.particles[i].weight])
    particles = new_particles
    last_heard_state_estimate = time.time()

def constructMessage(idin, status, accel, position, velocity, voltage, current, bat_temp, pod_temp, stripe_count):
    pattern = '!BB7iI'
    msg = struct.pack(pattern, idin, status, accel, position, velocity, voltage, current, bat_temp, pod_temp, stripe_count)
    return msg

def throw_estop(lc):
    msg = trigger_t()
    msg.utime = time.time() * 1000 * 1000
    lc.publish("_ESTOP", msg.encode())

def main():
    global fault_thrown, last_heard_estop
    global state, last_heard_state
    global particles, last_heard_state_estimate
    global high_power_battery_info, last_heard_high_battery
    global pod_temp, last_heard_pod_temp
    global stripe_count, last_heard_fiducial
    global brake_pos
    global last_heard_brake_state

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    lc = create_lcm()
    start_lcm(lc)

    state_sub = lc.subscribe("FSM_STATE", handle_state_t)
    fault_clear_sub = lc.subscribe("CLEAR_FAULT", handle_trigger_t)
    estop_sub = lc.subscribe("_ESTOP", handle_trigger_t)
    se_sub = lc.subscribe("_FC_SE", handle_state_estimate)

    keep_going = True
    last_send_time = time.time()
    while keep_going:
        try:
            now = time.time()
            elapsed = now -last_send_time
            if (elapsed > MESSAGE_SEND_PERIOD):
                last_send_time = now

                # update watchdogs
                if now - last_heard_state > WATCHDOG:
                    state = None
                    # faultable offense, as this is required telemetry
                    fault_thrown = True
                    throw_estop(lc)
                    print "FSM state timeout, faulting and estoping."

                if now - last_heard_state_estimate > WATCHDOG:
                    particles = None
                    # faultable offense, as this is required telemetry
                    fault_thrown = True
                    throw_estop(lc)
                    print "State estimate timeout, faulting and estoping."

                if now - last_heard_high_battery > WATCHDOG:
                    high_power_battery_info = None
                if now - last_heard_fiducial > WATCHDOG:
                    stripe_count = None
                if now - last_heard_pod_temp > WATCHDOG:
                    pod_temp = None
                if now - last_heard_brake_state > WATCHDOG:
                    brake_pos = None


                # update fault detector
                if state and state in [stmsg.ARM, stmsg.LAUNCH, stmsg.FLIGHT] and now - last_heard_estop < 0.5:
                    print "ESTOP overheard while in a flying state, fault must have happened!"
                    fault_thrown = True

                # decide all the things to send                
                state_to_send = STATUS_FAULT
                if not fault_thrown and state is not None:
                    if state == stmsg.ARM:
                        state_to_send = STATUS_READY
                    elif state == stmsg.LAUNCH:
                        state_to_send = STATUS_PUSHING
                    elif state == stmsg.FLIGHT and brake_pos and brake_pos < BRAKE_THRESHOLD:
                        state_to_send = STATUS_BRAKING
                    elif state == stmsg.FLIGHT:
                        state_to_send = STATUS_COAST
                    else:
                        state_to_send = STATUS_IDLE

                acc_to_send = 0
                vel_to_send = 0
                pos_to_send = 0    
                if (particles):
                    pos_to_send = int(particles[0][0][0]*100)
                    vel_to_send = int(particles[0][0][1]*100)
                    acc_to_send = int(particles[0][0][2]*100)

                voltage_to_send = 0
                current_to_send = 0
                bat_temp_to_send = 0
                if (high_power_battery_info):
                    # TODO: something smarter for multiple particles
                    voltage_to_send = int(high_power_battery_info[0]*1000)
                    current_to_send = int(high_power_battery_info[1]*1000)
                    bat_temp_to_send = int(high_power_battery_info[2]*10)

                pod_temp_to_send = 0
                if (pod_temp):
                    pod_temp_to_send = int(pod_temp*10)

                stripe_count_to_send = 0
                if (stripe_count and stripe_count > 0):
                    stripe_count_to_send = int(stripe_count)

                msg = constructMessage(MITHL_ID, state_to_send, acc_to_send, pos_to_send, vel_to_send, voltage_to_send, current_to_send, bat_temp_to_send, pod_temp_to_send, stripe_count_to_send)

                sock.sendto(msg, (TARGET_IP, TARGET_PORT))

                # and publish it to ourselves
                msg_lcm = spacex_telemetry_state_t()
                msg_lcm.utime = time.time() * 1000 * 1000
                msg_lcm.utime = MITHL_ID
                msg_lcm.status = state_to_send
                msg_lcm.acceleration = acc_to_send
                msg_lcm.position = pos_to_send
                msg_lcm.velocity = vel_to_send
                msg_lcm.battery_voltage = voltage_to_send
                msg_lcm.battery_current = current_to_send
                msg_lcm.battery_temperature = bat_temp_to_send
                msg_lcm.stripe_count = stripe_count_to_send
                lc.publish("SPACEX_RELAY_STATE", msg_lcm.encode())

        except KeyboardInterrupt:
            print "o7"
            break
        except Exception as e:
            print "Exception ", e

if __name__ == "__main__":
    main()
