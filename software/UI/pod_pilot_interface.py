#!/usr/bin/env python

''' Main pod pilot interface for MIT Hyperloop.
Invoke with 'python pod_pilot_interface.' '''

# the python stuff
import sys
import math
import signal
import thread
import os
import time
import random
import argparse

import numpy as np

# the interface stuff
from PyQt4 import QtCore, QtGui, QtOpenGL
import pyqtgraph as pg
from analog_vis import AnalogVisWidget, AnalogPlotWidget
from camera_vis import CameraVisWidget
from pod_driver import PodDriverWidget
from pod_tube_vis import PodTubeVisWidget
from netdiag_vis import NetDiagVisWidget
from sim_control import SimulationControlWidget
from teleop_panel import TeleopPanel
from braking_control import BrakingWidget
from brake_vis import BrakeVisWidget
from lcm_vis import LCMVisWidget
from mode_control_buttons import ModeControlUI
from pod_health_panel import PodHealthPanel
from pod_health_checker_panel import PodHealthCheckerPanel
from data_plot_widget import DataPlotWidget
from state_est_panel import StateEstPanel
from state_est_plot_panel import StateEstPlotPanel
from generic_message_sender import GenericMessageSender
from low_speed_panel import LowSpeedPanel
from mithl import flight_control_high_rate_t
from mithl import analog_rear_medium_rate_t
from mithl import analog_front_medium_rate_t

# the messaging stuff
import lcm
from lcm_utils import *
from mithl import brake_trajectory_simple_t, bac_config_t

#config
import yaml


# LAYOUT:
# |------------------------------|
# |      Core Health summary     |
# |-------------|----------------|
# |             |  Multipurpose  |
# |             |   Tab          |
# |             |   Panel        |
# |------------------------------|
# | Big Linear Display Everywhere|
# |------------------------------|
class Window(QtGui.QWidget):
    ''' The entire UI window. Everything lives in here.'''
    def __init__(self, config, lc):
        super(Window, self).__init__()

        self.lc = lc
        self.podHealthPanel = PodHealthPanel(lc=lc)
        self.podHealthCheckerPanel = PodHealthCheckerPanel("../config/healthConfig.yaml", lc=lc, name="Health Test")
        #self.podVisWidget = PodVisWidget(config, vis_config=vis_config, vis_config_file_path=vis_config_file_path, lc=lc)
        self.podTubeVisWidget = PodTubeVisWidget(config, lc=lc)
        #self.podDriverWidget = PodDriverWidget(config, lc=lc)
        self.IMUXPlotter = DataPlotWidget(2, title="Front and Rear Nav IMU X Acceleration, m/s/s", histLength=100, 
            dataRange=[-30, 30], alsoNumeric=True, name="IMU X", levelLines = [0.0])
        self.lc.subscribe("_FC_OH", self.handle_flight_control_high_rate)

        dataPlotsPanel = QtGui.QTabWidget()
        dataPlotsPanel.objectName = lambda: "Data Plots"
        
        gapHeightTareButton = QtGui.QPushButton('Tare')
        gapHeightTareButton.clicked.connect(self.ghTare)
        gapHeightTareButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)

        self.gapheight_plotter = DataPlotWidget(4, title="Ski Gap Height", histLength=1000, dataRange=[-30, 30], alsoNumeric=False, name="Ski Gap Height", levelLines = [0.0])
        self.lastAnalogRear = None
        self.lastAnalogFront = None
        self.gapHeightCalib = np.array([0., 0., 0., 0.])
        self.lc.subscribe("_AF_OM", self.handle_analog_front_med_rate)
        self.lc.subscribe("_AR_OM", self.handle_analog_rear_med_rate)

        ghLayout = QtGui.QVBoxLayout()
        ghLayout.addWidget(self.gapheight_plotter)
        ghLayout.addWidget(gapHeightTareButton)
        ghWidget = QtGui.QWidget()
        ghWidget.setLayout(ghLayout)

        dataPlotsPanel.addTab(
            ghWidget, "Gap Heights")

        detailedConfigsPanel = QtGui.QTabWidget()
        detailedConfigsPanel.objectName = lambda: "Configs"
        detailedConfigsPanel.addTab(
            GenericMessageSender(lc=lc, message_type=bac_config_t, 
                default_vals={
                    "utime" : 0,
                    "teleop_watchdog_duration" : 1.0,
                    "teleop_watchdog_used" : True,
                    "auto_watchdog_used" : True,
                    "auto_watchdog_duration": 1.0,
                    "minimum_disarm_duration": 5.0,
                    "valve_scaling_above" : 1.0,
                    "valve_scaling_below" : 1.0,
                    "valve_bias" : 0.82,
                    "valve_Kp" : 0.07,
                    "valve_Kd" : 0.025,
                    "valve_Ki" : 0.0,
                    "valve_exp" : 1.0,
                    "valve_min" : 0.7,
                    "motor_scaling" : 1.0,
                    "motor_bias" : 0.1,
                    "motor_Kp" : 1.0,
                    "motor_Kd" : 0.2,
                    "motor_Ki" : 0.0,
                    "motor_exp" : 1.0,
                    "integrator_lim" : 0.1,
                    "distance_estimate_rate_constant" : 0.05,
                    "velocity_estimate_rate_constant" : 0.1,
                    "deadzone_width" : 0.1,
                    "switch_period" : 0.25
                },
                limits_dict={
                    "valve_scaling_above": [0.0, 10.0],
                    "valve_scaling_below": [0.0, 10.0],
                    "motor_scaling": [0.0, 10.0],
                    "valve_bias": [0.0, 1.0],
                    "motor_bias": [0.0, 1.0],
                    "valve_Kp": [0.0, 100.0],
                    "valve_Kd": [0.0, 100.0],
                    "valve_Ki": [0.0, 100.0],
                    "valve_min": [0.0, 0.95],
                    "motor_Kp": [0.0, 100.0],
                    "motor_Kd": [0.0, 100.0],
                    "motor_Ki": [0.0, 100.0],
                    "integrator_lim": [0.0, 100.0]
                }, publish_channel="_BAC_CONFIG_SET", receive_channel="_BAC_CONFIG_STATE", name="BAC Config"
                ), "BAC Config")

        detailedConfigsPanel.addTab(
            GenericMessageSender(lc=lc, message_type=brake_trajectory_simple_t, 
                default_vals={
                        "utime" : 0,
                        "accel_alpha" : 0.1,
                        "accel_bias_alpha" : 0.0001,
                        "launch_start_accel_duration" : 0.4,
                        "launch_start_accel_threshold" : 0.2,
                        "launch_end_accel_duration" : 0.4,
                        "launch_end_accel_threshold" : 0.0,
                        "end_of_launch_timeout" : 10.0,
                        "cruise_setpoint" : 6,
                        "brake_trajectory_start_close_time" : 0,
                        "brake_trajectory_start_braking_speed": 1000,
                        "brake_trajectory_close_rate_inch_per_second": 99,
                        "flight_softstop_timeout" : 15.0,
                        "soft_stop_setpoint" : 0.0
                }, publish_channel="FSM_CONFIG_SET", receive_channel="FSM_CONFIG", name="Brake Traj"), "Brake Traj")

        podVisGridLayout = QtGui.QGridLayout()
        podVisGridLayout.addWidget(BrakeVisWidget(config, lc=lc, name="Brake Vis"), 0, 0, 1, 1)
        podVisGridLayout.addWidget(LCMVisWidget(config, lc=lc, name="LCM Vis"), 1, 0, 1, 1)
        totalPodVisWidget = QtGui.QWidget()
        totalPodVisWidget.setLayout(podVisGridLayout)
        self.tabWidgetLeft = QtGui.QTabWidget()
        self.tabsLeft = [
            totalPodVisWidget,
            CameraVisWidget("WEBCAM_F", lc=lc, name="CamF"),
            CameraVisWidget("WEBCAM_R", lc=lc, name="CamR")
            ]
        for tab in self.tabsLeft:
            self.tabWidgetLeft.addTab(tab, tab.objectName())

        self.tabWidgetRight = QtGui.QTabWidget()
        self.tabsRight = [
            ModeControlUI(lc, name="Mode"),
            TeleopPanel(config, lc=lc, name="Teleop"),
            LowSpeedPanel(config, lc=lc, name="Low Speed"),
            CameraVisWidget("WEBCAM_F", lc=lc, name="CamF"),
            CameraVisWidget("WEBCAM_R", lc=lc, name="CamR"),
            StateEstPanel(lc=lc, name="Nav"),
            StateEstPlotPanel(lc=lc, name="Nav Plot"),
            self.IMUXPlotter,
            NetDiagVisWidget(lc=lc, name="Net"), 
            self.podHealthCheckerPanel,
            detailedConfigsPanel,
            dataPlotsPanel,
            SimulationControlWidget(config, lc=lc, name="Sim")
        ]

        for tab in self.tabsRight:
            self.tabWidgetRight.addTab(tab, tab.objectName())

        mainLayout = QtGui.QGridLayout()

        mainLayout.addWidget(self.podHealthPanel, 0, 0, 1, 2)
        mainLayout.addWidget(self.tabWidgetLeft, 1, 0, 2, 1)
        mainLayout.addWidget(self.tabWidgetRight, 1, 1, 2, 1)
        mainLayout.addWidget(self.podTubeVisWidget, 3, 0, 1, 2)

        self.setLayout(mainLayout)

        self.setWindowTitle("MITHL Pod Pilot Interface")
        self.lastPanel = 0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

    def update(self):
        now = time.time()
        data = np.array([-10., -10., -10., -10.])
        if (self.lastAnalogFront is not None):
            data[0] = self.lastAnalogFront.ski_front_left_gh - self.gapHeightCalib[0]
            data[1] = self.lastAnalogFront.ski_front_right_gh - self.gapHeightCalib[1]
        if (self.lastAnalogRear is not None):
            data[2] = self.lastAnalogRear.ski_rear_left_gh - self.gapHeightCalib[2]
            data[3] = self.lastAnalogRear.ski_rear_right_gh - self.gapHeightCalib[3]
        for i in range(4):
            self.gapheight_plotter.addDataPoint(i, now, data[i])


    def ghTare(self):
        if (self.lastAnalogFront is not None):
            self.gapHeightCalib[0] = self.lastAnalogFront.ski_front_left_gh
            self.gapHeightCalib[1] = self.lastAnalogFront.ski_front_right_gh
        if (self.lastAnalogRear is not None):
            self.gapHeightCalib[2] = self.lastAnalogRear.ski_rear_left_gh
            self.gapHeightCalib[3] = self.lastAnalogRear.ski_rear_right_gh


    def handle_flight_control_high_rate(self, channel, data):
        msg = flight_control_high_rate_t.decode(data)
        self.IMUXPlotter.addDataPoint(0, float(msg.utime)/1000/1000, msg.front_nav_imu_xdd)
        self.IMUXPlotter.addDataPoint(1, float(msg.utime)/1000/1000, -msg.rear_nav_imu_xdd)

    def handle_analog_rear_med_rate(self, channel, data):
        msg = analog_rear_medium_rate_t.decode(data)
        self.lastAnalogRear = msg

    def handle_analog_front_med_rate(self, channel, data):
        msg = analog_front_medium_rate_t.decode(data)
        self.lastAnalogFront = msg

def main():
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", help="run in demo mode?", type=bool, default=False)
    args = parser.parse_args()

    lc = create_lcm()
    
    with open(os.path.realpath('../config/simConfig.yaml'), 'r') as f:
        config = yaml.load(f)
        
    #vis_config_file = "../models/pod.yaml"
    #vis_config_file_path = os.path.abspath(os.path.dirname(vis_config_file))
    #with open(vis_config_file, 'r') as f:
    #    vis_config = yaml.load(f)

    app 	= QtGui.QApplication(sys.argv)
    mainWindow = Window(config, lc)

    try:
        mainWindow.show()
    except KeyboardInterrupt:
        print "Interrupted"
    
    start_lcm(lc)
    sys.exit(app.exec_())   


if __name__ == '__main__':
    main()
