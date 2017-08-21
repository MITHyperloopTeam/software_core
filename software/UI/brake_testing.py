#!/usr/bin/env python

#signif reference to http://pastebin.com/k87sfiEf


import sys
import math
import signal
import time 
import os

import math, random
import numpy as np
from numpy import linalg
from PIL import Image

#interface stuff
from PyQt4 import QtCore, QtGui, QtOpenGL
import pyqtgraph as pg

#comms stuff
import lcm
from mithl import bac_auto_cmd_t
from mithl import bac_teleop_cmd_t
from mithl import bac_mode_t
from mithl import bac_config_t
from mithl import bac_state_high_rate_t
from mithl import bac_state_low_rate_t
from mithl import trigger_t
from lcm_utils import *

# what actually drives this
from generic_message_sender import GenericMessageSender

from data_plot_widget import DataPlotWidget

#read yaml config information
import yaml

distancesPlotter = []
estimatedDistancePlotter = []
effortsPlotter = []
errorPlotter = []
integratorPlotter = []
pressurePlotter = []

last_msg_plot_time = None

def handle_bac_state_high_rate_t(channel, data):
    global last_msg_plot_time
    msg = bac_state_high_rate_t.decode(data)
    
    t = msg.utime / 1000. / 1000.
    if (not last_msg_plot_time or t - last_msg_plot_time > 0.05):
        distancesPlotter.addDataPoint(0, t, msg.distance_1)
        distancesPlotter.addDataPoint(1, t, msg.distance_2)
        estimatedDistancePlotter.addDataPoint(0, t, msg.estimated_distance)
        pressurePlotter.addDataPoint(0, t, msg.brake_pressure)

        errorPlotter.addDataPoint(0, t, msg.error)
        integratorPlotter.addDataPoint(0, t, msg.integrator)

        effortsPlotter.addDataPoint(0, t, msg.PumpMotorPWM)
        effortsPlotter.addDataPoint(1, t, msg.ValvePWM)
        last_msg_plot_time = t

def handle_bac_state_low_rate_t(channel, data):
    msg = bac_state_low_rate_t.decode(data)
    # do something eventually?

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)

    window = QtGui.QWidget()
    
    windowLayout = QtGui.QHBoxLayout()

    controlPanelLayout = QtGui.QVBoxLayout()

    controlPanelLayout.addWidget( 
        GenericMessageSender(lc=lc, message_type=bac_auto_cmd_t, 
            default_vals={
                "utime" : 0,
                "setpoint" : 0.0
            },
            limits_dict={
                "setpoint": [0, 20.0]
            }, publish_channel="_BAC_AUTO_CMD_SET", receive_channel="_BAC_AUTO_CMD_STATE",
            spam_period=0.1))

    tabWidget = QtGui.QTabWidget()
    
    tabWidget.addTab( GenericMessageSender(lc=lc, message_type=bac_mode_t, 
            default_vals={
                "utime" : 0,
                "mode" : 0
            }, publish_channel="_BAC_MODE_SET", receive_channel="_BAC_MODE_STATE",
            options_dict={
                "mode": ["MODE_ESTOP", "MODE_TELEOP", "MODE_AUTO"]
            }) , "Mode Set")
        
    tabWidget.addTab( GenericMessageSender(lc=lc, message_type=bac_config_t, 
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
            }, publish_channel="_BAC_CONFIG_SET", receive_channel="_BAC_CONFIG_STATE",
            ), "Config")



    tabWidget.addTab(
        GenericMessageSender(lc=lc, message_type=bac_teleop_cmd_t, 
            default_vals={
                "utime" : 0,
                "PWREN" : False,
                "PumpMotorPWM" : 0.0,
                "PumpMotorEnable" : False,
                "OnOffValve" : False,
                "ValvePWM" : 0.0,
                "ValveEnable" : False,
                "LSMotorEnable" : False,
                "LSMotorDisable" : True,
                "LSActuatorEnable" : False,
                "LSActuatorDisable" : True
            }, publish_channel="_BAC_TELEOP_CMD_SET", receive_channel="_BAC_TELEOP_CMD_STATE",
            limits_dict={
                "PumpMotorPWM": [0., 1.],
                "ValvePWM" : [0., 1.]
            }, spam_period=0.1), "Teleop")

    controlPanelLayout.addWidget(tabWidget)

    controlPanel = QtGui.QWidget()
    controlPanel.setLayout(controlPanelLayout)
    windowLayout.addWidget(controlPanel)

    plotsLayout = QtGui.QVBoxLayout()

    distancesPlotter = DataPlotWidget(2, title="Distances", dataRange=[-1, 5], alsoNumeric=True)
    estimatedDistancePlotter = DataPlotWidget(1, title="Total Dist", dataRange=[0, 20], alsoNumeric=True)
    effortsPlotter = DataPlotWidget(2, title="Efforts", dataRange=[0, 1])
    errorPlotter = DataPlotWidget(1, title="Position Error", dataRange=[-3, 3])
    integratorPlotter = DataPlotWidget(1, title="Integrator", dataRange=[-1.1, 1.1])
    pressurePlotter = DataPlotWidget(1, title="Pressure", alsoNumeric=True, avgWindow = 0.05, dataRange=[0, 1500])
    
    distancesLayout = QtGui.QHBoxLayout()
    distancesLayout.addWidget(distancesPlotter)
    distancesLayout.addWidget(estimatedDistancePlotter)

    errorsLayout = QtGui.QHBoxLayout()
    errorsLayout.addWidget(errorPlotter)
    errorsLayout.addWidget(integratorPlotter)

    plotsLayout.addLayout(distancesLayout)
    plotsLayout.addWidget(effortsPlotter)
    plotsLayout.addLayout(errorsLayout)
    plotsLayout.addWidget(pressurePlotter)


    windowLayout.addLayout(plotsLayout)
    window.setLayout(windowLayout)
    window.show()

    lc.subscribe("_BAC_STATE_H", handle_bac_state_high_rate_t)
    lc.subscribe("_BAC_STATE_L", handle_bac_state_low_rate_t)

    start_lcm(lc)


    sys.exit(app.exec_())