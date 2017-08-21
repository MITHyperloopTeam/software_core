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

from generic_message_sender import GenericMessageSender

#comms stuff
import lcm
from mithl import vectorXf_t
from mithl import trigger_t
from mithl import velocity_t
from mithl import bac_mode_t, bac_teleop_cmd_t, bac_auto_cmd_t
from lcm_utils import *

#read yaml config information
import yaml

LABEL_DEFAULT_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 0px; border-color: black;"
LABEL_DISPLAY_GOOD_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: green; color: white"
LABEL_DISPLAY_WARN_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: blue; color: white"
LABEL_DISPLAY_PANIC_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: red; color: white"
LAYOUT_PANIC_STYLE_SHEET = "background-color: red"
LAYOUT_GOOD_STYLE_SHEET = "background-color: none"
TITLE_DEFAULT_STYLE_SHEET = "font: 24pt"


class TeleopPanel(QtGui.QWidget):
    ''' Intended features: Enables both direct and semi-automated control of BAC.
        When FSM is in teleop, or is not heard at all, allows:
           Button press control to switch BAC Mode to ESTOP, TELEOP SAFE, TELEOP HOLD, and AUTO SAFE, AUTO CURRENT. 
           --> switching TELEOP SAFE sets teleop settings to all off, and starts teleop.
           --> switching TELEOP HOLD sets teleop settings to prop valve and on/off valve closed, all other off.
           --> AUTO SAFE sets BAC setpoint to closed.
           --> AUTO CURRENT sets BAC setpoint to current measured brake openness.

           Teleop: holds hidden copy of "current commanded" teleop command that is spammed. "Commit" copies displayed
           to hidden commanded version. Macros set visible and hidden simultaneously. 

           Auto: same idea of committed and hidden.
            '''
    def __init__(self, config, lc=None, parent=None, name=None):
        super(TeleopPanel, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        vLayout = QtGui.QVBoxLayout()

        topLayout = QtGui.QGridLayout()
        labelMode = QtGui.QLabel()
        labelMode.setText("Mode Setting")
        labelMode.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
        labelMode.setAlignment(QtCore.Qt.AlignCenter)
        topLayout.addWidget(labelMode, 0, 0, 1, 1)
        topLayout.addWidget(
            GenericMessageSender(lc=lc, message_type=bac_mode_t, 
                default_vals={
                    "utime" : 0,
                    "mode" : 0
                }, publish_channel="_BAC_MODE_SET", receive_channel="_BAC_MODE_STATE",
                options_dict={
                    "mode": ["MODE_ESTOP", "MODE_TELEOP", "MODE_AUTO"]
                }), 1, 0, 1, 1)
        labelAuto = QtGui.QLabel()
        labelAuto.setText("Auto Setting")
        labelAuto.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
        labelAuto.setAlignment(QtCore.Qt.AlignCenter)
        topLayout.addWidget(labelAuto, 0, 1, 1, 1)
        topLayout.addWidget(
            GenericMessageSender(lc=lc, message_type=bac_auto_cmd_t, 
                default_vals={
                    "utime" : 0,
                    "setpoint" : 0.0
                },
                limits_dict={
                    "setpoint": [0, 20.0]
                }, publish_channel="_BAC_AUTO_CMD_SET", receive_channel="_BAC_AUTO_CMD_STATE",
                spam_period=0.1), 1, 1, 1, 1)
        vLayout.addLayout(topLayout)
        labelAuto = QtGui.QLabel()
        labelAuto.setText("Teleop Setting")
        labelAuto.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
        labelAuto.setAlignment(QtCore.Qt.AlignCenter)
        vLayout.addWidget(labelAuto)
        vLayout.addWidget(
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
                }, spam_period=0.1))

        self.setLayout(vLayout)

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = TeleopPanel(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
