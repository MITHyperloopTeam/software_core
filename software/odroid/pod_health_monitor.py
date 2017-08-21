#!/usr/bin/env python

import lcm
import lcm_utils
import time
import sys
import yaml
from threading import Lock
import math
import mithl
import numpy as np
from collections import namedtuple
from mithl import net_health_t, flight_control_high_rate_t, state_t, bac_config_t, trigger_t

class RateCheckInfo():
    ''' Resp Level of -1 = warn
        Resp level of 0 = nothing wrong
        Resp level of 1 = soft stop
        Resp level of 2 = estop '''

    def __init__(self, config, lc):
        self.channel = config["channel"]
        self.target_frequency = config["target_frequency"]
        self.count = 0
        self.bytes = 0
        self.frequency = 0.0
        self.resp_level = -1
        if "resp_level" in config.keys():
            self.resp_level = config["resp_level"]
        lc.subscribe(config["channel"], self.rate_check_handler)

    def update(self, dt):
        self.frequency = self.count / dt
        self.bandwidth = self.bytes / dt
        self.bytes = 0
        self.count = 0

    def get_bandwidth(self):
        return self.bandwidth

    def get_violation(self, frequency_droop_allowed):
        violation = self.resp_level * (self.frequency < math.floor(self.target_frequency*frequency_droop_allowed))
        if violation == 0:
            explanation = "%s rate %3.1fhz > %3.1fhz*%3.2f" % (self.channel, self.frequency, self.target_frequency, frequency_droop_allowed)
        else:
            explanation = "%s rate %3.1fhz < %3.1fhz*%3.2f" % (self.channel, self.frequency, self.target_frequency, frequency_droop_allowed)
        return violation, explanation

    def rate_check_handler(self, channel, data):
        try:
            self.count += 1
            self.bytes += len(data)
        except Exception as e:
            print "Exception in rate_check_handler ", self.channel, ":", e

class SensorCheckInfo():
    ''' Resp Level of -1 = warn
        Resp level of 0 = nothing wrong
        Resp level of 1 = soft stop
        Resp level of 2 = estop '''

    def __init__(self, config, lc):
        self.channel = config["channel"]
        self.type = getattr(mithl,config["type"])
        self.name = config["name"]
        self.min = config["range"][0]
        self.max = config["range"][1]
        self.noise_upper = 1E10
        if "noise_upper" in config.keys():
            self.noise_upper = config["noise_upper"]
        self.noise_lower = 1E-10
        if "noise_lower" in config.keys():
            self.noise_lower = config["noise_lower"]

        self.use_window_mean = False
        if "use_window_mean" in config.keys():
            self.use_window_mean = config["use_window_mean"]

        self.field_str = config["field_str"]

        self.history = np.array([0.0]*10)
        self.mean_est = 0
        self.var_est = 0
        self.var_good = False # isn't good estimate until we do one complete sweep
        self.history_ptr = 0

        self.count = 0
        self.violation = 0
        self.explanation = None
        self.resp_level = -1
        if "resp_level" in config.keys():
            self.resp_level = config["resp_level"]
        lc.subscribe(config["channel"], self.sensor_check_handler)

    def update(self, dt):
        pass

    def get_mean(self):
        if self.count == 0:
            return "???", self.resp_level
        else:
            if (self.mean_est < self.min or self.mean_est > self.max):
                return float(self.mean_est), self.resp_level
            else:
                return float(self.mean_est), 0

    def get_var(self):
        if self.count == 0:
            return "???", self.resp_level
        else:
            if (self.var_est < self.noise_lower or self.var_est > self.noise_upper):
                return float(self.var_est), self.resp_level
            else:
                return float(self.var_est), 0

    def get_violation(self):
        if self.count == 0:
            violation = self.resp_level
            explanation = "%s has not reported" % (self.channel)
        else: 
            violation = self.violation
            explanation = self.explanation
        return violation, explanation

    def sensor_check_handler(self, channel, data):
        try:
            self.count += 1
            # extract type and field
            msg = self.type.decode(data)
            x = eval("msg." + self.field_str)

            # update variance estimate
            old_x = self.history[self.history_ptr]
            self.history[self.history_ptr] = x
            self.history_ptr = (self.history_ptr + 1) % len(self.history)
            if (self.history_ptr == 0):
                self.var_good = True
            old_mean = self.mean_est
            self.mean_est += (x - old_x) / float(len(self.history))
            self.var_est += (x - old_x) * (x - self.mean_est + old_x - old_mean) / float(len(self.history) - 1)
            

            # do bounds checking
            x_compare = x
            if self.use_window_mean:
                x_compare = self.mean_est

            if self.mean_est < self.min:
                self.violation = self.resp_level
                self.explanation = "%s value %f < %f" % (self.name, self.mean_est, self.min)
            elif self.mean_est > self.max:
                self.violation = self.resp_level
                self.explanation = "%s value %f > %f" % (self.name, self.mean_est, self.max)
            elif self.var_est > self.noise_upper and self.var_good:
                self.violation = self.resp_level
                self.explanation = "%s variance %f > %f" % (self.name, self.var_est, self.noise_upper)
            elif self.var_est < self.noise_lower and self.var_good:
                self.violation = self.resp_level
                self.explanation = "%s variance %f < %f" % (self.name, self.var_est, self.noise_lower)


        except Exception as e:
            print "Exception in rate_check_handler ", self.channel, ":", e

class NetCheckInfo():
    def __init__(self, config, lc):
        self.lc = lc

        self.name_to_ind = {}
        for key, value in config["ids"].iteritems():
            self.name_to_ind[key] = value

        self.NetCheckItem = namedtuple('NetCheckItem', ('id_1', 'id_2', 'max_rtt', 'max_drop_rate', 'resp_level', 'name_1', 'name_2'))
        self.required_performances = []
        for entry in config["required_performances"]:
            self.required_performances.append(self.NetCheckItem( id_1=self.name_to_ind[entry["pair"][0]], 
                                                            id_2=self.name_to_ind[entry["pair"][1]],
                                                            max_rtt=entry["max_rtt"],
                                                            max_drop_rate=entry["max_drop_rate"],
                                                            resp_level=entry['resp_level'],
                                                            name_1 = entry["pair"][0],
                                                            name_2 = entry["pair"][1]))
        
        self.node_to_ind_map = {}
        self.ind_to_node_list = []
        self.next_ind = 0

        self.AdjacencyInfo = namedtuple('AdjacencyInfo', ('est_rtt', 'est_drop_rate', 'time'))
        self.adjacency_info_map = {}

        self.adjacency_labels = {}
        self.node_labels = {}

        nethealth_sub = self.lc.subscribe("_NSUM", self.handle_net_health_t)
        self.update_mutex = Lock()

    def update(self, dt):
        pass

    def get_rtt(self, item):
        if (item.id_1 not in self.node_to_ind_map.keys()) or (item.id_2 not in self.node_to_ind_map.keys()):
            return "???", item.resp_level

        ind1 = self.node_to_ind_map[item.id_1]
        ind2 = self.node_to_ind_map[item.id_2]

        if ((ind1, ind2) not in self.adjacency_info_map.keys()) or ((ind2, ind1) not in self.adjacency_info_map.keys()):
           return "???", item.resp_level
        else:
            info1 = self.adjacency_info_map[(ind1, ind2)]
            info2 = self.adjacency_info_map[(ind2, ind1)]
            if time.time() - info1.time > 5.0 or time.time() - info2.time > 5.0:
                return str("stale"), item.resp_level

            worst = max(info1.est_rtt, info2.est_rtt)
            if (worst > item.max_rtt):
                return str(worst), item.resp_level
            else:
                return str(worst), 0

    def get_drop_rate(self, item):
        if item.id_1 not in self.node_to_ind_map.keys() or item.id_2 not in self.node_to_ind_map.keys():
            return "???", item.resp_level

        ind1 = self.node_to_ind_map[item.id_1]
        ind2 = self.node_to_ind_map[item.id_2]

        if (ind1, ind2) not in self.adjacency_info_map.keys() or (ind2, ind1) not in self.adjacency_info_map.keys():
           return "???", item.resp_level
        else:
            info1 = self.adjacency_info_map[(ind1, ind2)]
            info2 = self.adjacency_info_map[(ind2, ind1)]
            if time.time() - info1.time > 5.0 or time.time() - info2.time > 5.0:
                return str("stale"), item.resp_level

            worst = max(info1.est_drop_rate, info2.est_drop_rate)
            if (worst > item.max_drop_rate):
                return str(worst), item.resp_level
            else:
                return str(worst), 0

        

    def test_performance(self, item):
        fail = False
        if item.id_1 not in self.node_to_ind_map.keys() or item.id_2 not in self.node_to_ind_map.keys():
            fail = True
            explanation = "(%d, %d) not known nodes in net" %  (item.id_1, item.id_2)
        else:
            ind1 = self.node_to_ind_map[item.id_1]
            ind2 = self.node_to_ind_map[item.id_2]
            if (ind1, ind2) not in self.adjacency_info_map.keys() and (ind2, ind1) not in self.adjacency_info_map.keys():
                fail = True
                explanation = "(%d, %d) not adjacent in net" % (item.id_1, item.id_2)
            else:
                info1 = self.adjacency_info_map[(ind1, ind2)]
                info2 = self.adjacency_info_map[(ind2, ind1)]
                if time.time() - info1.time > 5.0:
                    fail = True
                    explanation = "(%d, %d) adjacency stale in net" % (item.id_1, item.id_2)
                elif time.time() - info2.time > 5.0:
                    fail = True
                    explanation = "(%d, %d) adjacency stale in net" % (item.id_2, item.id_1)
                elif info1.est_rtt > item.max_rtt:
                    fail = True
                    explanation = "(%d, %d) rtt %f > max rtt %f" % (item.id_1, item.id_2, info1.est_rtt, item.max_rtt)
                elif info2.est_rtt > item.max_rtt:
                    fail = True
                    explanation = "(%d, %d) rtt %f > max rtt %f" % (item.id_2, item.id_1, info2.est_rtt, item.max_rtt)
                elif info1.est_drop_rate > item.max_drop_rate:
                    fail = True
                    explanation = "(%d, %d) rtt %f > max drop rate %f" % (item.id_1, item.id_2, info1.est_drop_rate, item.max_drop_rate)
                elif info2.est_drop_rate > item.max_drop_rate:
                    fail = True
                    explanation = "(%d, %d) rtt %f > max drop rate %f" % (item.id_2, item.id_1, info2.est_drop_rate, item.max_drop_rate)
                else:
                    explanation = ""
        if (fail):
            return item.resp_level, explanation
        else:
            return 0, explanation

    def get_violation(self):
        violation = 0
        explanation = []
        
        self.update_mutex.acquire()
        try:        
          for item in self.required_performances:
            newviol, newexp = self.test_performance(item)
            if (newviol):
                explanation.append(newexp)
                if (violation == 0):
                    violation = newviol
                else:
                    violation = max(violation, newviol)

        except Exception as e:
            print "Exception ", e, " in net checker"
        self.update_mutex.release()

        return violation, ",".join(explanation)


    def handle_net_health_t(self, channel, data):
        msg = net_health_t.decode(data)

        self.update_mutex.acquire()
        try:
          for host_i in range(msg.num_pairs):
            if msg.pair_transport_id[host_i] == 0:
                if msg.host_a[host_i] not in self.node_to_ind_map.keys():
                    # insert! record node positions:
                    self.node_to_ind_map[msg.host_a[host_i]] = self.next_ind
                    self.ind_to_node_list.append(msg.host_a[host_i])
                    # increase size of adjacency matrices:
                    self.next_ind += 1
                if msg.host_b[host_i] not in self.node_to_ind_map.keys():
                    # insert! record node positions:
                    self.node_to_ind_map[msg.host_b[host_i]] = self.next_ind
                    self.ind_to_node_list.append(msg.host_b[host_i])
                    # increase size of adjacency matrices:
                    self.next_ind += 1

                ind_a = self.node_to_ind_map[msg.host_a[host_i]]
                ind_b = self.node_to_ind_map[msg.host_b[host_i]]

                edge_info = self.AdjacencyInfo(est_rtt=msg.est_rtt[host_i], est_drop_rate=msg.est_drop_rate[host_i], time=time.time())
                self.adjacency_info_map[(ind_a, ind_b)] = edge_info
	except Exception as e:
          print "Exception ", e, " in net checker callback"
        self.update_mutex.release()


class ControlCheckInfo():

    def __init__(self, config, lc):
        self.max_imu_oscillations = config["max_imu_oscillations"]
        var_samples = config["variance_compute_samples"]
        self.oscillations_resp_level = config["oscillations_resp_level"]
        self.watchdog_resp_level = config["watchdog_resp_level"]
        self.history = np.array([[0.0]*var_samples]*6)
        self.mean_est = np.array([0.0]*6)
        self.var_est = np.array([0.0]*6)
        self.var_good = False # isn't good estimate until we do one complete sweep
        self.history_ptr = 0

        self.state = None
        self.auto_and_teleop_watchdogs_off = False

        lc.subscribe("_FC_OH", self.handle_flight_control_high_rate_t)
        lc.subscribe("FSM_STATE", self.handle_state_t)
        lc.subscribe("_BAC_CONFIG_STATE", self.handle_bac_config_t)

    def update(self, dt):
        pass

    def get_oscillations(self):
        if (self.var_good):
            osc = np.max(np.abs(self.var_est))
            if (osc > self.max_imu_oscillations):
                return str(osc), self.oscillations_resp_level
            else:
                return str(osc), 0
        else:
            return "???", self.oscillations_resp_level

    def get_watchdogs(self):
        if not self.state:
            return "No FSM State", 0
        elif (self.state != "ESTOP" and self.state != "TELEOP") and self.auto_and_teleop_watchdogs_off:
            return "OFF", self.watchdog_resp_level
        elif self.auto_and_teleop_watchdogs_off:
            return "OFF", -1
        else:
            return "ON", 0

    def get_violation(self):
        violation = 0
        explanation = None
        if self.var_good and np.max(np.abs(self.var_est)) > self.max_imu_oscillations:
            violation = self.oscillations_resp_level
            explanation = "Variance of %f in FC IMU signals" % np.max(np.abs(self.var_est))
        elif (self.state != "ESTOP" and self.state != "TELEOP") and self.auto_and_teleop_watchdogs_off:
            violation = self.watchdog_resp_level
            explanation = "Autonomous state being used without auto and teleop watchdogs"

    
        return violation, explanation

    def handle_flight_control_high_rate_t(self, channel, data):
        try:
            msg = flight_control_high_rate_t.decode(data)
             # update variance estimate
            x_front = np.array( [msg.front_nav_imu_xdd,
                msg.front_nav_imu_ydd,
                msg.front_nav_imu_zdd,
                msg.front_nav_imu_rd,
                msg.front_nav_imu_pd,
                msg.front_nav_imu_yd])
            x_rear = np.array( [msg.rear_nav_imu_xdd,
                msg.rear_nav_imu_ydd,
                msg.rear_nav_imu_zdd,
                msg.rear_nav_imu_rd,
                msg.rear_nav_imu_pd,
                msg.rear_nav_imu_yd])
            x = x_rear*0.5 + x_front*0.5
            old_x = self.history[:, self.history_ptr]
            self.history[:, self.history_ptr] = x
            self.history_ptr = (self.history_ptr + 1) % len(self.history)
            if (self.history_ptr == 0):
                self.var_good = True
            old_mean = self.mean_est
            self.mean_est += (x - old_x) / float(len(self.history))
            self.var_est += (x - old_x) * (x - self.mean_est + old_x - old_mean) / float(len(self.history) - 1)

        except Exception as e:
            print "Exception in FC OH handler ", channel, ":", e


    def handle_state_t(self, channel, data):
        try:
            msg = state_t.decode(data)
            if (msg.currentState == state_t.ESTOP):
                self.state = "ESTOP"
            elif (msg.currentState == state_t.TELEOP):
                self.state = "TELEOP"
            else:
                self.state = None
        except Exception as e:
            print "Exception in state handler ", channel, ":", e

    def handle_bac_config_t(self, channel, data):
        try:
            msg = bac_config_t.decode(data)
            if msg.teleop_watchdog_used and msg.auto_watchdog_used:
                self.auto_and_teleop_watchdogs_off = False
            else:
                self.auto_and_teleop_watchdogs_off = True

        except Exception as e:
            print "Exception in BAC config handler ", channel, ":", e


class PodHealthMonitor():
    ''' Monitors health of our overall system by subscribing to 
    message channels and checking the message rates and message
    contents.

        Initializes with a YAML file listing:
            - message rate checks to be performed
            - sensor values to be checked, by grouping

        Given a list of channel and message types, monitors the
            message channels and their rates, and 

        Given a list of MonitorInfos, 
        Given a list of channels, sensor fields, 
          - Net diag output
          - Power state
          - A bunch of sensors with ranges and typical noise
          - '''

    def __init__(self, configFile, lc):
        self.lc = lc
        config = yaml.load(open(configFile, 'r'))

        # NET CHECKS
        self.max_total_bandwidth = config["net_checks"]["max_total_bandwidth"]
        self.total_bandwidth = 0
        self.netChecker = NetCheckInfo(config["net_checks"], lc)
        self.net_check_last_update = time.time()

        # CONTROL CHECKS
        self.controlChecker = ControlCheckInfo(config["control_checks"], lc)
        self.control_check_last_update = time.time()

        # RATE CHECKS
        self.frequency_droop_allowed = config["message_rate_checks"]["frequency_droop_allowed"]
        self.rate_check_info_by_channel = {}
        for message_rate_check in config["message_rate_checks"]["items"]:
            if "resp_level" in message_rate_check.keys() and message_rate_check["resp_level"] != 0:
                self.rate_check_info_by_channel[message_rate_check["channel"]] = \
                    RateCheckInfo(message_rate_check, lc)
        self.rate_check_last_update = time.time()

        # SENSOR CHECKS
        self.sensor_check_groups = []
        for group in config["sensor_checks"]["items"].keys():
            sensor_check_info_by_name = {}
            for sensor_check in config["sensor_checks"]["items"][group]:
                if "resp_level" in sensor_check.keys() and sensor_check["resp_level"] != 0:
                    sensor_check_info_by_name[sensor_check["name"]] = \
                        SensorCheckInfo(sensor_check, lc)
            self.sensor_check_groups.append(sensor_check_info_by_name)
        self.sensor_check_last_update = time.time()


    def do_soft_stop(self):
        msg = state_t()
        msg.utime = time.time()*1000*1000
        msg.currentState = msg.SOFT_STOP
        self.lc.publish("FSM_REQUESTED_STATE", msg.encode())


    def do_estop(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish("_ESTOP", msg.encode())

    def update(self):
        requested_estops = []
        requested_soft_stops = []
        warns = [] 
        try:
            # RATE CHECKS
            elapsed = time.time() - self.rate_check_last_update
            if (elapsed > 3):
                self.total_bandwidth = 0
                self.rate_check_last_update = time.time()
                for key, value in self.rate_check_info_by_channel.iteritems():
                    value.update(elapsed)
                    violation, explanation = value.get_violation(self.frequency_droop_allowed)
                    self.total_bandwidth += value.get_bandwidth() * 8 / 1e+6
                    if violation == -1:
                        warns.append(explanation)
                    elif violation == 1:
                        requested_soft_stops.append(explanation)
                    elif violation == 2:
                        requested_estops.append(explanation)

                if self.total_bandwidth > self.max_total_bandwidth:
                    requested_soft_stops.append("Current bandwidth %f > limit %f" % (self.total_bandwidth, self.max_total_bandwidth))

            # SENSOR CHECKS
            elapsed = time.time() - self.sensor_check_last_update
            if (elapsed > 1.0):
                self.sensor_check_last_update = time.time()
                for group in self.sensor_check_groups:
                    for key, value in group.iteritems():
                        value.update(elapsed)
                        violation, explanation = value.get_violation()
                        if violation == -1:
                            warns.append(explanation)
                        elif violation == 1:
                            requested_soft_stops.append(explanation)
                        elif violation == 2:
                            requested_estops.append(explanation)

            # NET CHECKS
            elapsed = time.time() - self.net_check_last_update
            if (elapsed > 1.0):
                self.net_check_last_update = time.time()
                violation, explanation = self.netChecker.get_violation()
                if (violation == -1):
                    warns.append(explanation)
                elif violation == 1:
                    requested_soft_stops.append(explanation)
                elif violation == 2:
                    requested_estops.append(explanation)

            # CONTROL CHECKS
            elapsed = time.time() - self.control_check_last_update
            if (elapsed > 1.0):
                self.control_check_last_update = time.time()
                violation, explanation = self.controlChecker.get_violation()
                if (violation == -1):
                    warns.append(explanation)
                elif violation == 1:
                    requested_soft_stops.append(explanation)
                elif violation == 2:
                    requested_estops.append(explanation)


        except Exception as e:
            print "EXCPETION"
            requested_estops.append("Exception in health monitor: " + str(e))

        highest_offense = 0
        if len(warns):
            highest_offense = -1
            print "WARNS:"
            for i in warns:
                print "\t*", i
        if len(requested_soft_stops):
            highest_offense = 1
            print "SOFTSTOPS:"
            for i in requested_soft_stops:
                print "\t*", i
        if len(requested_estops):
            highest_offense = 2
            print "ESTOPS:"
            for i in requested_estops:
                print "\t*", i

        if (highest_offense == 1):
            self.do_soft_stop()
        elif (highest_offense == 2):
            self.do_estop()


if __name__ == '__main__':
    lc = lcm_utils.create_lcm()
    phm = PodHealthMonitor(sys.argv[1], lc = lc)

    lcm_utils.start_lcm(lc)
    going = True
    while (going):
        try:
            phm.update()
        except KeyboardInterrupt:
            going = False
	time.sleep(0.0001)
    print "Bye o7"
        
