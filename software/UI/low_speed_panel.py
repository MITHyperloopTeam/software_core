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
from mithl import vectorXf_t
from mithl import trigger_t
from mithl import adm_config_t
from mithl import adm_status_t
from lcm_utils import *

from copy import deepcopy


from threading import Lock

#read yaml config information
import yaml

LABEL_DEFAULT_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 0px; border-color: black;"
LABEL_DISPLAY_GOOD_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: green; color: white"
LABEL_DISPLAY_WARN_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: blue; color: white"
LABEL_DISPLAY_PANIC_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: red; color: white"
LAYOUT_PANIC_STYLE_SHEET = "background-color: red"
LAYOUT_GOOD_STYLE_SHEET = "background-color: none"
TITLE_DEFAULT_STYLE_SHEET = "font: 24pt"


class RoboteqADMWidget(QtGui.QWidget):
    ''' Low Speed Management Widget '''
    def __init__(self, driver_name, lc=None, parent=None, name=None):
        super(RoboteqADMWidget, self).__init__(parent)

        self.config_set_channel = "_RADM_%s_SETCONF" % driver_name
        self.config_cur_channel = "_RADM_%s_CURCONF" % driver_name
        self.status_cur_channel = "_RADM_%s_STATUS" % driver_name
        self.reflash_channel = "_RADM_%s_REFLASH" % driver_name
        self.estop_channel = "_ESTOP"

        self.COMMAND_SEND_PERIOD = 0.2

        self.last_known_status = None
        self.status_mtx = Lock()

        # set storage of things
        self.last_known_config = {
            "estop": -2,
            "command": 0,
            "mode": 0,
            "current_lim": 40.0,
            "Kp": 20.0,
            "Kd": 20.0,
            "Ki": 20.0,
            "integral_cap": 100,
            "max_rpm": 5040,
            "max_power_forward": 100,
            "max_power_reverse": 100,
            "n_poles": 8,
            "encoder_usage": 2,
            "switching_mode": 2,
            "sinusoidal_mode": 2,
            "encoder_pulse_per_rev": 500,
            "encoder_low_count_limit": -100000,
            "encoder_high_count_limit": 100000,
            "default_pos": 0.0,
            "default_vel": 0.0,
            "default_current": 0.0,
            "closed_loop_error_detection": 2,
            "closed_loop_feedback_sensor": 1
        }
        self.tentative_config = deepcopy(self.last_known_config)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        lsLayout = QtGui.QVBoxLayout()

        self.force_estop = True
        estopLayout = QtGui.QGridLayout()
        self.estopButton = QtGui.QPushButton('ESTOP', self)
        self.estopButton.clicked.connect(self.handleEstopButton)
        self.estopButton.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
        self.resetEstopButton = QtGui.QPushButton('(Reset Estop)', self)
        self.resetEstopButton.clicked.connect(self.handleResetEstopButton)
        estopLayout.addWidget(self.estopButton, 0, 0, 1, 1)
        estopLayout.addWidget(self.resetEstopButton, 0, 1, 1, 1)
        self.estopWidgetLabel = QtGui.QLabel()
        self.estopWidgetLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.estopWidgetLabel.setStyleSheet(TITLE_DEFAULT_STYLE_SHEET)
        self.estopWidgetLabel.setText("Trying to ESTOP")
        estopLayout.addWidget(self.estopWidgetLabel, 1, 0, 1, 2)

        
        # for convenience of setup and maintenance, store the
        # label, the interface element setter, and interface element getter
        self.configValueFunctionLabelPairs = []

        # Command subpanel
        self.active_command_mode = False
        self.commandLayout = QtGui.QVBoxLayout()
        self.commandHLayout = QtGui.QHBoxLayout()
        commandLabel = QtGui.QLabel("Command: ")
        self.last_sent_command = time.time()
        self.commandSlider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.commandSlider.setValue(0)
        self.commandSlider.setMinimum(-1000)
        self.commandSlider.setMaximum(1000)
        self.commandSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.commandSlider.setTickInterval(100)
        self.commandSlider.setSingleStep(1)
        #self.commandSlider.setMinimumWidth(200)

        self.commandSlider.valueChanged.connect(self.updateAllConfig)
        self.commandSliderLabel = QtGui.QLabel("??")
        self.commandSliderLabel.setMinimumWidth(50) 
        self.commandActiveModeToggleBtn = QtGui.QPushButton()
        self.commandActiveModeToggleBtn.setText("ENABLE Active Command")
        self.commandActiveModeToggleBtn.clicked.connect(self.toggleActiveCommandMode)


        def commandSliderSyncer():
            self.commandSliderLabel.setText(str(self.commandSlider.value()/1000.0))
        def commandUpdater(x):
            self.commandSlider.setValue(x*1000)
            self.commandSliderLabel.setText(str(x))

        self.commandZeroBtn = QtGui.QPushButton()
        self.commandZeroBtn.setText("Reset Cmd to Zero")
        self.commandZeroBtn.clicked.connect(lambda: commandUpdater(0.0))

        self.commandHLayout.addWidget(commandLabel)
        self.commandHLayout.addWidget(self.commandSlider)
        self.commandHLayout.addWidget(self.commandSliderLabel)
        self.commandLayout.addLayout(self.commandHLayout)
        self.commandLayout.addWidget(self.commandActiveModeToggleBtn)
        self.commandLayout.addWidget(self.commandZeroBtn)

        self.commandSlider.valueChanged.connect(commandSliderSyncer)
        self.configValueFunctionLabelPairs.append(("command", commandLabel, lambda:self.commandSlider.value()/1000.0, commandUpdater))

        # Configuration subpanel
        configLayout = QtGui.QVBoxLayout()

        opModeLayout = QtGui.QHBoxLayout()
        opModeLabel = QtGui.QLabel('Op Mode:')
        self.opModeComboBox =  QtGui.QComboBox(self)
        # prepending with numbers to force correct ordering
        # we should move to string search maybe? 
        self.opModes = ["0: Open-Loop Speed",
                        "1: Closed-Loop Speed",
                        "2: Closed-Loop Position Relative",
                        "3: Closed-Loop Count Position",
                        "4: Closed-Loop Position Tracking",
                        "5: Torque"]
        for state in self.opModes:
            self.opModeComboBox.addItem(state)
        self.opModeComboBox.activated[str].connect(self.updateAllConfig)
        opModeLayout.addWidget(opModeLabel)
        opModeLayout.addWidget(self.opModeComboBox)
        self.configValueFunctionLabelPairs.append(("mode", opModeLabel, self.opModeComboBox.currentIndex, self.opModeComboBox.setCurrentIndex))
        configLayout.addLayout(opModeLayout)

        CLEDLayout = QtGui.QHBoxLayout()
        CLEDLabel = QtGui.QLabel('CLED Mode:')
        self.CLEDComboBox =  QtGui.QComboBox(self)
        # prepending with numbers to force correct ordering
        # we should move to string search maybe? 
        self.CLEDs = ["0: None",
                        "1: 250ms",
                        "2: 500ms",
                        "3: 1000ms"]
        for state in self.CLEDs:
            self.CLEDComboBox.addItem(state)
        self.CLEDComboBox.activated[str].connect(self.updateAllConfig)
        CLEDLayout.addWidget(CLEDLabel)
        CLEDLayout.addWidget(self.CLEDComboBox)
        self.configValueFunctionLabelPairs.append(("closed_loop_error_detection", CLEDLabel, self.CLEDComboBox.currentIndex, self.CLEDComboBox.setCurrentIndex))
        configLayout.addLayout(CLEDLayout)

        CLSensorLayout = QtGui.QHBoxLayout()
        CLSensor = QtGui.QLabel('CL Sensor:')
        self.CLSensorComboBox =  QtGui.QComboBox(self)
        # prepending with numbers to force correct ordering
        # we should move to string search maybe? 
        self.CLSensors = ["0: Hall",
                          "1: Encoder"]
        for state in self.CLSensors:
            self.CLSensorComboBox.addItem(state)
        self.CLSensorComboBox.activated[str].connect(self.updateAllConfig)
        CLSensorLayout.addWidget(CLSensor)
        CLSensorLayout.addWidget(self.CLSensorComboBox)
        self.configValueFunctionLabelPairs.append(("closed_loop_feedback_sensor", CLSensor, self.CLSensorComboBox.currentIndex, self.CLSensorComboBox.setCurrentIndex))
        configLayout.addLayout(CLSensorLayout)

        currentLimitLayout = QtGui.QHBoxLayout()
        currentLimitLabel = QtGui.QLabel('Current Lim:')
        self.currentLimitSpinbox = QtGui.QDoubleSpinBox()
        self.currentLimitSpinbox.setRange(0., 50.)
        self.currentLimitSpinbox.setValue(40.)
        self.currentLimitSpinbox.setSuffix('A')
        self.currentLimitSpinbox.valueChanged.connect(self.updateAllConfig)
        currentLimitLayout.addWidget(currentLimitLabel)
        currentLimitLayout.addWidget(self.currentLimitSpinbox)
        self.configValueFunctionLabelPairs.append(("current_lim", currentLimitLabel, self.currentLimitSpinbox.value, self.currentLimitSpinbox.setValue))
        configLayout.addLayout(currentLimitLayout)

        PIDGainsLayout = QtGui.QHBoxLayout()
        PIDGainsKpLabel = QtGui.QLabel('Kp:')
        self.PIDGainsKpSpinbox = QtGui.QDoubleSpinBox()
        self.PIDGainsKpSpinbox.setRange(0, 25)
        self.PIDGainsKpSpinbox.setValue(20.)
        self.PIDGainsKpSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("Kp", PIDGainsKpLabel, self.PIDGainsKpSpinbox.value, self.PIDGainsKpSpinbox.setValue))
        PIDGainsKdLabel = QtGui.QLabel('Kd:')
        self.PIDGainsKdSpinbox = QtGui.QDoubleSpinBox()
        self.PIDGainsKdSpinbox.setRange(0, 25)
        self.PIDGainsKdSpinbox.setValue(20.)
        self.PIDGainsKdSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("Kd", PIDGainsKdLabel, self.PIDGainsKdSpinbox.value, self.PIDGainsKdSpinbox.setValue))
        PIDGainsKiLabel = QtGui.QLabel('Ki:')
        self.PIDGainsKiSpinbox = QtGui.QDoubleSpinBox()
        self.PIDGainsKiSpinbox.setRange(0, 25)
        self.PIDGainsKiSpinbox.setValue(20.)
        self.PIDGainsKiSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("Ki", PIDGainsKiLabel, self.PIDGainsKiSpinbox.value, self.PIDGainsKiSpinbox.setValue))
        PIDGainsLayout.addWidget(PIDGainsKpLabel)
        PIDGainsLayout.addWidget(self.PIDGainsKpSpinbox)
        PIDGainsLayout.addWidget(PIDGainsKdLabel)
        PIDGainsLayout.addWidget(self.PIDGainsKdSpinbox)
        PIDGainsLayout.addWidget(PIDGainsKiLabel)
        PIDGainsLayout.addWidget(self.PIDGainsKiSpinbox)
        configLayout.addLayout(PIDGainsLayout)

        RPMLayout = QtGui.QHBoxLayout()
        MaxRPMLabel = QtGui.QLabel('Max RPM:')
        self.MaxRPMSpinbox = QtGui.QSpinBox()
        self.MaxRPMSpinbox.setRange(10, 65000)
        self.MaxRPMSpinbox.setValue(20)
        self.MaxRPMSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("max_rpm", MaxRPMLabel, self.MaxRPMSpinbox.value, self.MaxRPMSpinbox.setValue))
        RPMLayout.addWidget(MaxRPMLabel)
        RPMLayout.addWidget(self.MaxRPMSpinbox)
        configLayout.addLayout(RPMLayout)

        MaxPowerLayout = QtGui.QHBoxLayout()
        MaxPwrFwdLabel = QtGui.QLabel('Max Pwr Fwd:')
        self.MaxPwrFwdSpinbox = QtGui.QSpinBox()
        self.MaxPwrFwdSpinbox.setRange(0, 100)
        self.MaxPwrFwdSpinbox.setValue(100)
        self.MaxPwrFwdSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("max_power_forward", MaxPwrFwdLabel, self.MaxPwrFwdSpinbox.value, self.MaxPwrFwdSpinbox.setValue))
        MaxPwrRvsLabel = QtGui.QLabel('Max Pwr Rev:')
        self.MaxPwrRvsSpinbox = QtGui.QSpinBox()
        self.MaxPwrRvsSpinbox.setRange(0, 100)
        self.MaxPwrRvsSpinbox.setValue(100)
        self.MaxPwrRvsSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("max_power_reverse", MaxPwrRvsLabel, self.MaxPwrRvsSpinbox.value, self.MaxPwrRvsSpinbox.setValue))
        MaxPowerLayout.addWidget(MaxPwrFwdLabel)
        MaxPowerLayout.addWidget(self.MaxPwrFwdSpinbox)
        MaxPowerLayout.addWidget(MaxPwrRvsLabel)
        MaxPowerLayout.addWidget(self.MaxPwrRvsSpinbox)
        configLayout.addLayout(MaxPowerLayout)

        PolesLayout = QtGui.QHBoxLayout()
        PolesLabel = QtGui.QLabel('N Poles:')
        self.NPolesSpinbox = QtGui.QSpinBox()
        self.NPolesSpinbox.setRange(0, 99)
        self.NPolesSpinbox.setValue(8)
        self.NPolesSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("n_poles", PolesLabel, self.NPolesSpinbox.value, self.NPolesSpinbox.setValue))
        PolesLayout.addWidget(PolesLabel)
        PolesLayout.addWidget(self.NPolesSpinbox)
        configLayout.addLayout(PolesLayout)

        EncoderUsageLayout = QtGui.QHBoxLayout()
        EncoderUsageLabel = QtGui.QLabel('Encoder Usage:')
        self.EncoderUsageComboBox =  QtGui.QComboBox(self)
        # prepending with numbers to force correct ordering
        # we should move to string search maybe? 
        self.EncoderUsages = ["0: Unused",
                        "1: Command",
                        "2: Feedback"]
        for state in self.EncoderUsages:
            self.EncoderUsageComboBox.addItem(state)
        self.EncoderUsageComboBox.activated[str].connect(self.updateAllConfig)
        EncoderUsageLayout.addWidget(EncoderUsageLabel)
        EncoderUsageLayout.addWidget(self.EncoderUsageComboBox)
        self.configValueFunctionLabelPairs.append(("encoder_usage", EncoderUsageLabel, self.EncoderUsageComboBox.currentIndex, self.EncoderUsageComboBox.setCurrentIndex))
        configLayout.addLayout(EncoderUsageLayout)

        SwitchingModeLayout = QtGui.QHBoxLayout()
        SwitchingModeLabel = QtGui.QLabel('Switching Mode:')
        self.SwitchingModeComboBox =  QtGui.QComboBox(self)
        # prepending with numbers to force correct ordering
        # we should move to string search maybe? 
        self.SwitchingModes = ["0: Hall",
                        "1: Sinusoidal",
                        "2: Sensorless"]
        for state in self.SwitchingModes:
            self.SwitchingModeComboBox.addItem(state)
        self.SwitchingModeComboBox.activated[str].connect(self.updateAllConfig)
        SwitchingModeLayout.addWidget(SwitchingModeLabel)
        SwitchingModeLayout.addWidget(self.SwitchingModeComboBox)
        self.configValueFunctionLabelPairs.append(("switching_mode", SwitchingModeLabel, self.SwitchingModeComboBox.currentIndex, self.SwitchingModeComboBox.setCurrentIndex))
        configLayout.addLayout(SwitchingModeLayout)

        SinusoidalModeLayout = QtGui.QHBoxLayout()
        SinusoidalModeLabel = QtGui.QLabel('Sinusoidal Mode:')
        self.SinusoidalModeComboBox =  QtGui.QComboBox(self)
        # prepending with numbers to force correct ordering
        # we should move to string search maybe? 
        self.SinusoidalModes = ["0: Encoder",
                        "1: Hall",
                        "2: Hall+Encoder",
                        "3: SPI Sensor",
                        "4: Sin Cos Sensor",
                        "5: Resolver"]
        for state in self.SinusoidalModes:
            self.SinusoidalModeComboBox.addItem(state)
        self.SinusoidalModeComboBox.activated[str].connect(self.updateAllConfig)
        SinusoidalModeLayout.addWidget(SinusoidalModeLabel)
        SinusoidalModeLayout.addWidget(self.SinusoidalModeComboBox)
        self.configValueFunctionLabelPairs.append(("sinusoidal_mode", SinusoidalModeLabel, self.SinusoidalModeComboBox.currentIndex, self.SinusoidalModeComboBox.setCurrentIndex))
        configLayout.addLayout(SinusoidalModeLayout)

        PPRLayout = QtGui.QHBoxLayout()
        PPRLabel = QtGui.QLabel('Encoder Pulse per Rev:')
        self.PPRSpinbox = QtGui.QSpinBox()
        self.PPRSpinbox.setRange(-2, 100000)
        self.PPRSpinbox.setValue(500)
        self.PPRSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("encoder_pulse_per_rev", PPRLabel, self.PPRSpinbox.value, self.PPRSpinbox.setValue))
        PPRLayout.addWidget(PPRLabel)
        PPRLayout.addWidget(self.PPRSpinbox)
        configLayout.addLayout(PPRLayout)

        EncoderCountLimitLayout = QtGui.QHBoxLayout()
        HighLimitLabel = QtGui.QLabel('Enc H Limit:')
        self.HLimitSpinbox = QtGui.QSpinBox()
        self.HLimitSpinbox.setRange(-2, 10000000)
        self.HLimitSpinbox.setValue(100000)
        self.HLimitSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("encoder_high_count_limit", HighLimitLabel, self.HLimitSpinbox.value, self.HLimitSpinbox.setValue))
        LowLimitLabel = QtGui.QLabel('Enc L Limit:')
        self.LLimitSpinbox = QtGui.QSpinBox()
        self.LLimitSpinbox.setRange(-10000000, 0)
        self.LLimitSpinbox.setValue(100000)
        self.LLimitSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("encoder_low_count_limit", LowLimitLabel, self.LLimitSpinbox.value, self.LLimitSpinbox.setValue))
        EncoderCountLimitLayout.addWidget(LowLimitLabel)
        EncoderCountLimitLayout.addWidget(self.LLimitSpinbox)
        EncoderCountLimitLayout.addWidget(HighLimitLabel)
        EncoderCountLimitLayout.addWidget(self.HLimitSpinbox)
        configLayout.addLayout(EncoderCountLimitLayout)

        IntegralCapLayout = QtGui.QHBoxLayout()
        IntegralCapLabel = QtGui.QLabel('Integral Cap:')
        IntegralCapLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.IntegralCapSpinbox = QtGui.QSpinBox()
        self.IntegralCapSpinbox.setRange(0, 100)
        self.IntegralCapSpinbox.setValue(100)
        self.IntegralCapSpinbox.setSuffix('%')
        self.IntegralCapSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("integral_cap", IntegralCapLabel, self.IntegralCapSpinbox.value, self.IntegralCapSpinbox.setValue))
        IntegralCapLayout.addWidget(IntegralCapLabel)
        IntegralCapLayout.addWidget(self.IntegralCapSpinbox)
        configLayout.addLayout(IntegralCapLayout)

        DefaultsLayout = QtGui.QHBoxLayout()
        DefaultsPosLabel = QtGui.QLabel('Pos:')
        self.DefaultsPosSpinbox = QtGui.QDoubleSpinBox()
        self.DefaultsPosSpinbox.setRange(0, 25)
        self.DefaultsPosSpinbox.setValue(0.)
        self.DefaultsPosSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("default_pos", DefaultsPosLabel, self.DefaultsPosSpinbox.value, self.DefaultsPosSpinbox.setValue))
        DefaultsVelLabel = QtGui.QLabel('Vel:')
        self.DefaultsVelSpinbox = QtGui.QDoubleSpinBox()
        self.DefaultsVelSpinbox.setRange(0, 25)
        self.DefaultsVelSpinbox.setValue(0.)
        self.DefaultsVelSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("default_vel", DefaultsVelLabel, self.DefaultsVelSpinbox.value, self.DefaultsVelSpinbox.setValue))
        DefaultsCurrentLabel = QtGui.QLabel('Current:')
        self.DefaultsCurrentSpinbox = QtGui.QDoubleSpinBox()
        self.DefaultsCurrentSpinbox.setRange(0, 25)
        self.DefaultsCurrentSpinbox.setValue(0.)
        self.DefaultsCurrentSpinbox.valueChanged.connect(self.updateAllConfig)
        self.configValueFunctionLabelPairs.append(("default_current", DefaultsCurrentLabel, self.DefaultsCurrentSpinbox.value, self.DefaultsCurrentSpinbox.setValue))
        DefaultsLayout.addWidget(DefaultsPosLabel)
        DefaultsLayout.addWidget(self.DefaultsPosSpinbox)
        DefaultsLayout.addWidget(DefaultsVelLabel)
        DefaultsLayout.addWidget(self.DefaultsVelSpinbox)
        DefaultsLayout.addWidget(DefaultsCurrentLabel)
        DefaultsLayout.addWidget(self.DefaultsCurrentSpinbox)
        configLayout.addLayout(DefaultsLayout)

        configButtonsLayout = QtGui.QHBoxLayout()
        self.configSyncButton = QtGui.QPushButton('Sync', self)
        self.configSyncButton.clicked.connect(self.resetAllConfig)
        self.configCommitButton = QtGui.QPushButton('Commit', self)
        self.configCommitButton.clicked.connect(self.handleConfigCommitButton)
        self.reflashButton = QtGui.QPushButton('Reflash All', self)
        self.reflashButton.clicked.connect(self.handleReflashButton)
        configButtonsLayout.addWidget(self.configSyncButton)
        configButtonsLayout.addWidget(self.configCommitButton)
        configButtonsLayout.addWidget(self.reflashButton)

        configScroll = QtGui.QScrollArea()
        configScrollWidget = QtGui.QWidget()
        configScrollWidget.setLayout(configLayout)
        configScroll.setWidget(configScrollWidget)

        configOverallLayout = QtGui.QVBoxLayout()
        configOverallLayout.addWidget(configScroll)
        configOverallLayout.addLayout(configButtonsLayout)

        configWidget = QtGui.QWidget()
        configWidget.setLayout(configOverallLayout)
        configWidget.setStyleSheet("""
        .QWidget {
            border: 5px solid gray;
            border-radius: 5px;
            background-color: rgb(255, 255, 255);
            }
        """)

        for pair in self.configValueFunctionLabelPairs:
            pair[1].setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            pair[1].setAutoFillBackground(True)


        # Configuration subpanel
        statusLayout = QtGui.QGridLayout()

        motorCurrentNameLabel = QtGui.QLabel('Motor: ')
        self.motorCurrentLabel = QtGui.QLabel('00.0A')
        motorCurrentNameLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.motorCurrentLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        statusLayout.addWidget(motorCurrentNameLabel, 0, 0, 1, 1)
        statusLayout.addWidget(self.motorCurrentLabel, 0, 1, 1, 1)
        
        batteryCurrentNameLabel = QtGui.QLabel('Battery: ')
        self.batteryCurrentLabel = QtGui.QLabel('00.0A')
        batteryCurrentNameLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.batteryCurrentLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        statusLayout.addWidget(batteryCurrentNameLabel, 0, 2, 1, 1)
        statusLayout.addWidget(self.batteryCurrentLabel, 0, 3, 1, 1)

        tempNameLabel = QtGui.QLabel('Temp: ')
        self.tempLabel = QtGui.QLabel('00.0C')
        tempNameLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.tempLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        statusLayout.addWidget(tempNameLabel, 1, 0, 1, 1)
        statusLayout.addWidget(self.tempLabel, 1, 1, 1, 1)
        
        rpmNameLabel = QtGui.QLabel('RPM: ')
        self.rpmLabel = QtGui.QLabel('0000')
        rpmNameLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.rpmLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        statusLayout.addWidget(rpmNameLabel, 1, 2, 1, 1)
        statusLayout.addWidget(self.rpmLabel, 1, 3, 1, 1)

        voltageNameLabel = QtGui.QLabel('Voltage: ')
        self.voltageLabel = QtGui.QLabel('00.0V')
        voltageNameLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.voltageLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        statusLayout.addWidget(voltageNameLabel, 2, 1, 1, 1)
        statusLayout.addWidget(self.voltageLabel, 2, 2, 1, 1)

        self.statusWidget = QtGui.QWidget()
        self.statusWidget.setLayout(statusLayout)
        self.statusWidget.setStyleSheet("""
        .QWidget {
            border: 5px solid gray;
            border-radius: 5px;
            background-color: rgb(200, 200, 20);
            }
        """)

        self.estopWidget = QtGui.QWidget()
        self.estopWidget.setLayout(estopLayout)
        self.estopWidget.setStyleSheet("""
        .QWidget {
            border: 5px solid gray;
            border-radius: 5px;
            }
        """)

        lsLayout.addWidget(self.estopWidget)
        lsLayout.addWidget(self.statusWidget)
        lsLayout.addLayout(self.commandLayout)
        lsLayout.addWidget(configWidget)

        self.setLayout(lsLayout)

        # just to be sure...
        self.resetAllConfig()
        self.updateAllConfig() 

        configsub = self.lc.subscribe(self.config_cur_channel, self.handle_adm_config_t)
        statussub = self.lc.subscribe(self.status_cur_channel, self.handle_adm_status_t)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def update(self):
        if (self.last_known_status):
            if (self.last_known_status.current_battery[0] != -2):
                self.batteryCurrentLabel.setText( "%2.1fA" % self.last_known_status.current_battery[0])
            else:
                self.batteryCurrentLabel.setText( "??.?A" )
            if (self.last_known_status.current_motor[0] != -2):
                self.motorCurrentLabel.setText( "%2.1fA" % self.last_known_status.current_motor[0])
            else:
                self.motorCurrentLabel.setText( "??.?A" )
            if (self.last_known_status.voltage[1] != -2):
                self.voltageLabel.setText( "%2.1fV %2.1fV %2.1fV" % (self.last_known_status.voltage[0], self.last_known_status.voltage[1], self.last_known_status.voltage[2]))
            else:
                self.voltageLabel.setText( "??.?V ??.?V ??.?V" )
            if (self.last_known_status.temp[0] != -2):
                self.tempLabel.setText( "%2.1fC" % self.last_known_status.temp[0])
            else:
                self.tempLabel.setText( "??.?C" )
            if (self.last_known_status.rpm[0] != -2):
                self.rpmLabel.setText( "%4d" % self.last_known_status.rpm[0])
            else:
                self.rpmLabel.setText( "????" )

        if (self.force_estop == True):
            if (not self.last_known_config or self.last_known_config["estop"] != 1):
                self.estopWidgetLabel.setStyleSheet("""
                    color: white;
                    font: 14pt;
                    background-color: rgb(250, 20, 20)
                    """)
                self.estopWidgetLabel.setText("Trying to ESTOP")
            else:
                self.estopWidgetLabel.setStyleSheet("""
                    color: white;
                    font: 14pt;
                    background-color: rgb(20, 250, 20)
                    """)
                self.estopWidgetLabel.setText("ESTOP active")
        else:
            if (self.last_known_config and self.last_known_config["estop"] == 0):
                self.estopWidgetLabel.setStyleSheet("""
                    color: white;
                    font: 14pt;
                    background-color: rgb(200, 150, 20)
                    """)
                self.estopWidgetLabel.setText("ESTOP cleared")
            else:
                self.estopWidgetLabel.setStyleSheet("""
                    color: white;
                    font: 14pt;
                    background-color: rgb(20, 150, 200)
                    """)
                self.estopWidgetLabel.setText("Trying to clear ESTOP")

        if (self.active_command_mode):
            if (time.time() - self.last_sent_command > self.COMMAND_SEND_PERIOD):
                msg = adm_config_t()
                for attr in msg.__slots__:
                    setattr(msg, attr, -2)
                msg.utime = time.time()*1000*1000
                msg.command = self.commandSlider.value()/1000.0
                self.lc.publish(self.config_set_channel, msg.encode())
                self.last_sent_command = time.time()


    def updateAllConfig(self):
        # handle spinboxes
        nonmatch_palette = QtGui.QPalette()
        nonmatch_palette.setColor(QtGui.QLabel().backgroundRole(), QtGui.QColor(255, 0, 0, 50))

        match_palette = QtGui.QPalette()
        match_palette.setColor(QtGui.QLabel().backgroundRole(), QtCore.Qt.white)

        approx_equal = lambda a, b, t: abs(a - b) < t
        for pair in self.configValueFunctionLabelPairs:
            if not approx_equal(pair[2](),  self.last_known_config[pair[0]], 0.0001):
                pair[1].setPalette(nonmatch_palette)
            else:
                pair[1].setPalette(match_palette)


    def resetAllConfig(self):
        self.tentative_config = self.last_known_config
        
        for pair in self.configValueFunctionLabelPairs:
            pair[3]( self.tentative_config[pair[0]] )

        self.updateAllConfig()


    def toggleActiveCommandMode(self):
        if (self.active_command_mode):
            self.commandActiveModeToggleBtn.setText("ENABLE Active Command")
            self.active_command_mode = False
        else:
            self.active_command_mode = True
            self.commandActiveModeToggleBtn.setText("DISABLE Active Command")

    def handleEstopButton(self):
        self.force_estop = True
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish(self.estop_channel, msg.encode())

        
    def handleResetEstopButton(self):
        self.force_estop = False
        msg = adm_config_t()
        for attr in msg.__slots__:
            setattr(msg, attr, -2)
        msg.utime = time.time()*1000*1000
        msg.estop = 0
        self.lc.publish(self.config_set_channel, msg.encode())
        print "clicked", self.force_estop
        #self.estopWidget.setStyleSheet("""
        #    .QWidget {
        #        border: 5px solid gray;
        #        border-radius: 5px;
        #        background-color: rgb(20, 200, 200);
        #        }
        #    """)

        
    def handleConfigCommitButton(self):
        msg = adm_config_t()
        for attr in msg.__slots__:
            setattr(msg, attr, -2)
        msg.utime = time.time()*1000*1000
        for pair in self.configValueFunctionLabelPairs:
            if pair[2]() != self.last_known_config[pair[0]]:
                setattr(msg, pair[0], pair[2]())
            else:
                # no config options are strings so python autocasting will take care
                # of this
                setattr(msg, pair[0], -2)
        self.lc.publish(self.config_set_channel, msg.encode())

    def handleReflashButton(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish(self.reflash_channel, msg.encode())

    def handle_adm_config_t(self, channel, data):
        msg = adm_config_t.decode(data)
        for key in self.last_known_config.keys():
            try:
                newval = getattr(msg, key)
                self.last_known_config[key] = newval
            except Exception as e:
                print "Problem in parsing low speed config t: ", e
        self.updateAllConfig()


    def handle_adm_status_t(self, channel, data):
        msg = adm_status_t.decode(data)
        self.status_mtx.acquire()
        self.last_known_status = msg
        self.status_mtx.release()


class LowSpeedPanel(QtGui.QWidget):
    def __init__(self, config, lc=None, parent=None, name=None):
        super(LowSpeedPanel, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        hLayout = QtGui.QGridLayout()

        clamp_label = QtGui.QLabel("Clamp")
        clamp_label.setAlignment(QtCore.Qt.AlignCenter)
        clamp_label.setStyleSheet(TITLE_DEFAULT_STYLE_SHEET)
        wheel_label = QtGui.QLabel("Wheel")
        wheel_label.setAlignment(QtCore.Qt.AlignCenter)
        wheel_label.setStyleSheet(TITLE_DEFAULT_STYLE_SHEET)

        clamp_roboteq = RoboteqADMWidget("CLAMP", lc=lc)
        wheel_roboteq = RoboteqADMWidget("WHEEL", lc=lc)
        hLayout.addWidget(clamp_label, 0, 0, 1, 1)
        hLayout.addWidget(wheel_label, 0, 1, 1, 1)
        hLayout.addWidget(clamp_roboteq, 1, 0, 1, 1)
        hLayout.addWidget(wheel_roboteq, 1, 1, 1, 1)
        self.setLayout(hLayout)

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = LowSpeedPanel(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
