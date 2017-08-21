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
from threading import Lock

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

#read yaml config information
import yaml

# controller stuff
from mithl import gamepad_t


class RoboteqControllerDriver(QtGui.QWidget):
    ''' Drives low speed system from a gamepad '''
    def __init__(self, lc=None, parent=None, name=None):
        super(RoboteqControllerDriver, self).__init__(parent)
        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        self.CONTROLLER_WATCHDOG_TIMEOUT = 1.0
        self.COMMAND_SEND_PERIOD = 0.2

        # LAYOUT STARTS NOW
        layout = QtGui.QVBoxLayout()

        # Overall status
        self.status_label = QtGui.QLabel("NO CONTROLLER")
        layout.addWidget(self.status_label)

        # Whether this spams commands or not
        self.active_command_mode = False
        self.commandActiveModeToggleBtn = QtGui.QPushButton()
        self.commandActiveModeToggleBtn.setText("ENABLE Active Command")
        self.commandActiveModeToggleBtn.clicked.connect(self.toggleActiveCommandMode)
        self.last_sent_command = time.time() - self.COMMAND_SEND_PERIOD
        layout.addWidget(self.commandActiveModeToggleBtn)

        # ESTOP
        self.estop_label = QtGui.QLabel("Press B to ESTOP")
        layout.addWidget(self.estop_label)
        # ESTOP CONTROL
        self.estop_btn = 1 # "B", big red button

        # CLAMP
        linear_actuator_layout = QtGui.QHBoxLayout()
        linear_actuator_label = QtGui.QLabel("Lin Actuator: ")
        self.linear_actuator_num = QtGui.QLabel("0.00")
        linear_actuator_layout.addWidget(linear_actuator_label)
        linear_actuator_layout.addWidget(self.linear_actuator_num)
        layout.addLayout(linear_actuator_layout)
        # linear actuator on hat, stepping on every input
        self.linear_actuator_hat_i = 0
        self.linear_actuator_command = 0.0
        self.last_linear_actuator_hat_input = 0
        self.linear_actuator_step = 0.05

        # DRIVE
        drive_layout = QtGui.QHBoxLayout()
        drive_label = QtGui.QLabel("Drive Motor: ")
        self.drive_num_label = QtGui.QLabel("???")
        drive_layout.addWidget(drive_label)
        drive_layout.addWidget(self.drive_num_label)
        self.drive_label = QtGui.QLabel("")
        layout.addLayout(drive_layout)
        # DRIVE CONTROL
        self.drive_axis = 4 # right stick up/down
        self.drive_axis_scale = -0.2  # flip it to get direction right and dramatically reduce sensistivity     
        self.drive_deadzone = 0.1

        self.setLayout(layout)

        # CONTROLLER SUBSCRIPTION
        self.controller_msg_mtx = Lock()
        self.last_controller_config = None
        self.last_heard_from_controller = time.time() - self.CONTROLLER_WATCHDOG_TIMEOUT
        controllersub = self.lc.subscribe("GAMEPAD", self.handle_gamepad_t)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)


    def toggleActiveCommandMode(self):
        if (self.active_command_mode):
            self.commandActiveModeToggleBtn.setText("ENABLE Active Command")
            self.active_command_mode = False    
        else:
            self.active_command_mode = True
            self.commandActiveModeToggleBtn.setText("DISABLE Active Command")

    def update(self):
        self.controller_msg_mtx.acquire()

        if (self.last_controller_config and time.time() - self.last_heard_from_controller < self.CONTROLLER_WATCHDOG_TIMEOUT):
            self.status_label.setText("FOUND CONTROLLER")

            if (self.last_controller_config.button[self.estop_btn] != 0):
                self.sendEStop()
                self.linear_actuator_command = 0.0
                self.active_command_mode = False
                self.commandActiveModeToggleBtn.setText("ENABLE Active Command")
                self.estop_label.setStyleSheet("""
                .QWidget {
                    border: 5px solid gray;
                    border-radius: 5px;
                    background-color: rgb(250, 20, 20);
                    }
                """)
            else:
                self.estop_label.setStyleSheet("""
                .QWidget {
                    border: 5px solid gray;
                    border-radius: 5px;
                    background-color: rgb(20, 20, 20);
                    }
                """)


                # linear actuator:
                if (self.last_controller_config.hat_y[self.linear_actuator_hat_i] != self.last_linear_actuator_hat_input):
                    self.linear_actuator_command += self.linear_actuator_step * self.last_controller_config.hat_y[self.linear_actuator_hat_i]
                    self.last_linear_actuator_hat_input = self.last_controller_config.hat_y[self.linear_actuator_hat_i]

                self.linear_actuator_num.setText("%1.2f" % self.linear_actuator_command)

                axis_input_with_deadzone = self.last_controller_config.axis[self.drive_axis]
                if (abs(axis_input_with_deadzone) < self.drive_deadzone):
                    axis_input_with_deadzone = 0
                motor_command = axis_input_with_deadzone*self.drive_axis_scale
                self.drive_num_label.setText("%1.2f" % motor_command)

                if (self.active_command_mode):
                    if (time.time() - self.last_sent_command > self.COMMAND_SEND_PERIOD):
                        self.sendLowSpeedConfigWithCommand(self.linear_actuator_command, "_RADM_CLAMP_SETCONF")
                        self.sendLowSpeedConfigWithCommand(motor_command, "_RADM_WHEEL_SETCONF")
                        self.last_sent_command = time.time()

        else:
            self.status_label.setText("NO CONTROLLER")

        
        self.controller_msg_mtx.release()

    def handle_gamepad_t(self, channel, data):
        msg = gamepad_t.decode(data)
        self.controller_msg_mtx.acquire()
        self.last_controller_config = msg
        self.last_heard_from_controller = time.time()
        self.controller_msg_mtx.release()
        
    def sendLowSpeedConfigWithCommand(self, x, channel):
        msg = adm_config_t()
        for attr in msg.__slots__:
            setattr(msg, attr, -2)
        msg.utime = time.time()*1000*1000
        if (abs(x) < 0.0001):
            msg.command = 0
        else:
            msg.command = x
        self.lc.publish(channel, msg.encode())

    def sendEStop(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish("_ESTOP", msg.encode())


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
        estopLayout = QtGui.QHBoxLayout()
        self.estopButton = QtGui.QPushButton('ESTOP', self)
        self.estopButton.clicked.connect(self.handleEstopButton)
        self.resetEstopButton = QtGui.QPushButton('(Reset Estop)', self)
        self.resetEstopButton.clicked.connect(self.handleResetEstopButton)
        estopLayout.addWidget(self.estopButton)
        estopLayout.addWidget(self.resetEstopButton)

        
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
        configLayout.addLayout(configButtonsLayout)

        configWidget = QtGui.QWidget()
        configWidget.setLayout(configLayout)
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
        statusLayout = QtGui.QVBoxLayout()

        motorCurrentLayout = QtGui.QHBoxLayout()
        motorCurrentNameLabel = QtGui.QLabel('Motor: ')
        self.motorCurrentLabel = QtGui.QLabel('00.0A')
        self.motorCurrentLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        motorCurrentLayout.addWidget(motorCurrentNameLabel)
        motorCurrentLayout.addWidget(self.motorCurrentLabel)
        statusLayout.addLayout(motorCurrentLayout)

        batteryCurrentLayout = QtGui.QHBoxLayout()
        batteryCurrentNameLabel = QtGui.QLabel('Battery: ')
        self.batteryCurrentLabel = QtGui.QLabel('00.0A')
        self.batteryCurrentLabel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        batteryCurrentLayout.addWidget(batteryCurrentNameLabel)
        batteryCurrentLayout.addWidget(self.batteryCurrentLabel)
        statusLayout.addLayout(batteryCurrentLayout)


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
            background-color: rgb(200, 200, 20);
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
        self.update()


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

        if (self.force_estop):
            if (not self.last_known_config or self.last_known_config["estop"] != 1):
                msg = trigger_t()
                msg.utime = time.time()*1000*1000
                self.lc.publish(self.estop_channel, msg.encode())
                self.estopWidget.setStyleSheet("""
                    .QWidget {
                        border: 5px solid gray;
                        border-radius: 5px;
                        background-color: rgb(250, 20, 20);
                        }
                    """)
            else:
                self.estopWidget.setStyleSheet("""
                    .QWidget {
                        border: 5px solid gray;
                        border-radius: 5px;
                        background-color: rgb(230, 150, 20);
                        }
                    """)
        else:
            if (self.last_known_config and self.last_known_config["estop"] == 0):
                self.estopWidget.setStyleSheet("""
                    .QWidget {
                        border: 5px solid gray;
                        border-radius: 5px;
                        background-color: rgb(20, 200, 20);
                        }
                    """)
            elif (self.last_known_config and self.last_known_config["estop"] == 1):
                self.estopWidget.setStyleSheet("""
                    .QWidget {
                        border: 5px solid gray;
                        border-radius: 5px;
                        background-color: rgb(250, 20, 20);
                        }
                    """)

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
        
    def handleResetEstopButton(self):
        self.force_estop = False
        msg = adm_config_t()
        for attr in msg.__slots__:
            setattr(msg, attr, -2)
        msg.utime = time.time()*1000*1000
        msg.estop = 0
        self.lc.publish(self.config_set_channel, msg.encode())
        self.estopWidget.setStyleSheet("""
            .QWidget {
                border: 5px solid gray;
                border-radius: 5px;
                background-color: rgb(20, 200, 200);
                }
            """)

        
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
                if (newval != -2):
                    self.last_known_config[key] = newval
            except Exception as e:
                print "Problem in parsing low speed config t: ", e
        self.updateAllConfig()


    def handle_adm_status_t(self, channel, data):
        msg = adm_status_t.decode(data)
        self.status_mtx.acquire()
        self.last_known_status = msg
        self.status_mtx.release()

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    overallWidget = QtGui.QWidget()
    overallLayout = QtGui.QVBoxLayout()

    controllerWidget = RoboteqControllerDriver(lc=lc)

    clamp_roboteq = RoboteqADMWidget("CLAMP", lc=lc)
    wheel_roboteq = RoboteqADMWidget("WHEEL", lc=lc)
    tabwidget = QtGui.QTabWidget()
    tabwidget.addTab(clamp_roboteq, "CLAMP")
    tabwidget.addTab(wheel_roboteq, "WHEEL")
    
    overallLayout.addWidget(controllerWidget)
    overallLayout.addWidget(tabwidget)
    overallWidget.setLayout(overallLayout)
    overallWidget.show()

    start_lcm(lc)

    sys.exit(app.exec_())