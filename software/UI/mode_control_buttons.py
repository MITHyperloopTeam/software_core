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
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

#comms stuff
import lcm
from mithl import state_t, bac_config_t, vectorXf_t, fsm_state_high_rate_t, trigger_t
from lcm_utils import *

from data_plot_widget import DataPlotWidget

class ModeUIModel(object):
    WATCHDOG = 5.0

    def __init__(self, lc, code_to_name_dict):
        self.lc = lc
        self.code_to_name_dict = code_to_name_dict

        self.latest_state_t = None
        self.latest_bac_config_t = None
        self.latest_bac_countdown = None
        self.last_heard_state = time.time() - self.WATCHDOG
        self.last_heard_bac_config = time.time() - self.WATCHDOG
        self.last_heard_arm_countdown = time.time() - self.WATCHDOG
        self.last_heard_fsm_state_high_rate = time.time() - self.WATCHDOG
        self.last_est_accel = None
        #Subscribe to mode
        self.lc.subscribe("FSM_STATE", self.new_state_handler)
        self.lc.subscribe("_BAC_CONFIG_STATE", self.new_config_handler) # bac_config_t minimum_disarm_duration
        self.lc.subscribe("_BAC_ARM_COUNTDOWN", self.new_arm_timeout_handler) # bac_config_t minimum_disarm_duration
        self.lc.subscribe("FSM_OH", self.fsm_oh_handler)

    def fsm_oh_handler(self, channel, data):
        msg = fsm_state_high_rate_t.decode(data)
        self.last_est_accel = msg.acceleration_est
        self.last_heard_fsm_state_high_rate = time.time()

    def new_state_handler(self, channel, data):
        msg = state_t.decode(data)
        self.latest_state_t = msg
        self.last_heard_state = time.time()
    
    def new_config_handler(self, channel, data):
        msg = bac_config_t.decode(data)
        self.latest_bac_config_t = msg
        self.last_heard_bac_config = time.time()

    def new_arm_timeout_handler(self, channel, data):
        msg = vectorXf_t.decode(data)
        if (msg.rows > 0):
            self.latest_bac_countdown = msg.data[0]
            self.last_heard_arm_countdown = time.time()

class ModeUIView(QtGui.QWidget):
    def __init__(self, lc, model, code_to_name_dict, hist_length=10, parent=None):
        super(QtGui.QWidget, self).__init__(parent)

        self.lc = lc

        self.model = model
        self.code_to_name_dict = code_to_name_dict

        self.view_layout = QtGui.QGridLayout(self)

        # History plotter 
        self.curr_mode = -1
        self.t0 = time.time()
        modeHistoryBoxLayout = QtGui.QGridLayout()
        modeHistoryBoxLayout.addWidget(QtGui.QLabel('Mode'), 0, 0, 1, 1)
        modeHistoryBoxLayout.addWidget(QtGui.QLabel('Start Time'), 0, 1, 1, 1)
        self.modeHistoryLabels = []
        for i in range(hist_length):
            modeNameBox = QtGui.QLabel("<>")
            modeTBox = QtGui.QLabel("0.0")
            modeHistoryBoxLayout.addWidget(modeNameBox, i+1, 0, 1, 1)
            modeHistoryBoxLayout.addWidget(modeTBox, i+1, 1, 1, 1)
            self.modeHistoryLabels.append((modeNameBox, modeTBox))
        modeHistoryBox = QtGui.QWidget()
        modeHistoryBox.setLayout(modeHistoryBoxLayout)
        self.view_layout.addWidget(modeHistoryBox, 6, 1, 5, 1)
        
        armPeriodViewHeader = QtGui.QLabel('Current Minimum Disarm Duration:')
        armPeriodViewHeader.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = armPeriodViewHeader.styleSheet()
        s += "font: 12pt;"
        armPeriodViewHeader.setStyleSheet(s)
        self.armPeriodViewHeader = armPeriodViewHeader
        self.view_layout.addWidget(armPeriodViewHeader, 0, 0, 1, 2)
        
        armPeriodView = QtGui.QLabel('##nostate##')
        armPeriodView.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = armPeriodView.styleSheet()
        s += "font: 40pt;"
        s += "background-color: rgb(0,50,0); color: rgb(0,230,0);"
        armPeriodView.setStyleSheet(s)
        armPeriodView.setAlignment(QtCore.Qt.AlignCenter);
        self.view_layout.addWidget(armPeriodView, 1, 1, 1, 1)
        self.armPeriodView = armPeriodView

        armLockoutView = QtGui.QLabel('???')
        armLockoutView.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = armLockoutView.styleSheet()
        s += "font: 40pt;"
        s += "background-color: rgb(0,50,0); color: rgb(0,230,0);"
        armLockoutView.setStyleSheet(s)
        armLockoutView.setAlignment(QtCore.Qt.AlignCenter);
        self.view_layout.addWidget(armLockoutView, 1, 0, 1, 1)
        self.armLockoutView = armLockoutView

        fsmStateViewHeader = QtGui.QLabel('Current State:')
        fsmStateViewHeader.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = fsmStateViewHeader.styleSheet()
        s += "font: 12pt;"
        fsmStateViewHeader.setStyleSheet(s)
        self.fsmStateViewHeader = fsmStateViewHeader
        self.view_layout.addWidget(fsmStateViewHeader, 2, 0, 1, 2)
        
        fsmStateView = QtGui.QLabel('##nostate##')
        fsmStateView.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = fsmStateView.styleSheet()
        s += "font: 40pt;"
        s += "background-color: rgb(0,50,0); color: rgb(0,230,0);"
        fsmStateView.setStyleSheet(s)
        fsmStateView.setAlignment(QtCore.Qt.AlignCenter);
        self.view_layout.addWidget(fsmStateView, 3, 0, 1, 2)
        self.fsmStateView = fsmStateView
        
        self.fsmAccView = DataPlotWidget(1, title="FSM Est Acc", histLength=300, alsoNumeric=False, name="FSM Est Acc", levelLines = [0.0])
        self.fsmAccView.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        self.view_layout.addWidget(self.fsmAccView, 4, 0, 1, 2)

        tareIMUButton = QtGui.QPushButton('Tare Est Acceleration')
        tareIMUButton.clicked.connect(self.sendTare)
        tareIMUButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = tareIMUButton.styleSheet()
        s += "font: 24pt;"
        tareIMUButton.setStyleSheet(s)
        self.view_layout.addWidget(tareIMUButton, 5, 1, 1, 1)
        self.tareIMUButton = tareIMUButton


        estopButton = QtGui.QPushButton('ESTOP')
        estopButton.clicked.connect(self.sendEStop)
        estopButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = estopButton.styleSheet()
        s += "font: 36pt;"
        s += "background-color: {:s}; color: white;".format("Red")
        estopButton.setStyleSheet(s)
        self.view_layout.addWidget(estopButton, 5, 0, 1, 1)
        self.estopButton = estopButton
        
        shutdownButton = QtGui.QPushButton('Shutdown')
        shutdownButton.clicked.connect(self.sendShutdown)
        shutdownButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = shutdownButton.styleSheet()
        s += "font: 36pt;"
        #s += "background-color: {:s}; color: white;".format("Orange")
        shutdownButton.setStyleSheet(s)
        self.view_layout.addWidget(shutdownButton, 6, 0, 1, 1)
        self.shutdownButton = shutdownButton

        softStopButton = QtGui.QPushButton('Soft Stop')
        softStopButton.clicked.connect(self.sendSoftStop)
        softStopButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = softStopButton.styleSheet()
        s += "font: 36pt;"
        s += "background-color: {:s}; color: white;".format("Orange")
        softStopButton.setStyleSheet(s)
        self.view_layout.addWidget(softStopButton, 7, 0, 1, 1)
        self.softStopButton = softStopButton
        
        armButton = QtGui.QPushButton('Arm')
        armButton.clicked.connect(self.sendArming)
        armButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = armButton.styleSheet()
        s += "font: 36pt;"
        #s += "background-color: {:s}; color: white;".format("Orange")
        armButton.setStyleSheet(s)
        self.view_layout.addWidget(armButton, 8, 0, 1, 1)
        self.armButton = armButton
        
        #driveButton = QtGui.QPushButton('Drive')
        #driveButton.clicked.connect(self.sendPreDrive)
        #driveButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        #s = driveButton.styleSheet()
        #s += "font: 36pt;"
        #s += "background-color: {:s}; color: white;".format("Orange")
        #driveButton.setStyleSheet(s)
        #self.view_layout.addWidget(driveButton, 8, 0, 1, 1)
        #self.driveButton = driveButton
        

        teleopButton = QtGui.QPushButton('Teleop')
        teleopButton.clicked.connect(self.sendTeleop)
        teleopButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = teleopButton.styleSheet()
        s += "font: 36pt;"
        #s += "background-color: {:s}; color: white;".format("Orange")
        teleopButton.setStyleSheet(s)
        self.view_layout.addWidget(teleopButton, 9, 0, 1, 1)
        self.teleopButton = teleopButton

        self.setLayout(self.view_layout)
        
        self.timer= QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)
        

    def sendTare(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish("TARE", msg.encode())

    def sendEStop(self):
        msg = state_t()
        msg.utime = time.time()*1000*1000
        msg.currentState = msg.ESTOP
        self.lc.publish("FSM_REQUESTED_STATE", msg.encode())

    def sendShutdown(self):
        msg = state_t()
        msg.utime = time.time()*1000*1000
        msg.currentState = msg.SHUTDOWN
        self.lc.publish("FSM_REQUESTED_STATE", msg.encode())

    def sendSoftStop(self):
        msg = state_t()
        msg.utime = time.time()*1000*1000
        msg.currentState = msg.SOFT_STOP
        self.lc.publish("FSM_REQUESTED_STATE", msg.encode())

    def sendArming(self):
        msg = state_t()
        msg.utime = time.time()*1000*1000
        msg.currentState = msg.ARMING
        self.lc.publish("FSM_REQUESTED_STATE", msg.encode())

    def sendPreDrive(self):
        msg = state_t()
        msg.utime = time.time()*1000*1000
        msg.currentState = msg.PRE_DRIVE
        self.lc.publish("FSM_REQUESTED_STATE", msg.encode())

    def sendTeleop(self):
        msg = state_t()
        msg.utime = time.time()*1000*1000
        msg.currentState = msg.TELEOP
        self.lc.publish("FSM_REQUESTED_STATE", msg.encode())

    def update(self):
        if (self.model.latest_state_t and time.time() - self.model.last_heard_state < self.model.WATCHDOG):
            state_code = self.model.latest_state_t.currentState
            state_code_text = ('##noncode %d ##' % (state_code)) if (state_code not in self.code_to_name_dict) else ('%d : %s' % (state_code, self.code_to_name_dict[state_code]))
            if (state_code != self.curr_mode):
                self.curr_mode = state_code
                for i in range(len(self.modeHistoryLabels)-1):
                    self.modeHistoryLabels[i][0].setText(self.modeHistoryLabels[i+1][0].text())
                    self.modeHistoryLabels[i][1].setText(self.modeHistoryLabels[i+1][1].text())
                self.modeHistoryLabels[-1][0].setText(state_code_text)
                self.modeHistoryLabels[-1][1].setText("%5.3f" % (time.time()-self.t0))

            self.fsmStateView.setText(state_code_text)

        if (self.model.latest_bac_config_t and time.time() - self.model.last_heard_bac_config < self.model.WATCHDOG):
            min_arm = self.model.latest_bac_config_t.minimum_disarm_duration
            self.armPeriodView.setText('(Max %f)' % min_arm)
        else:
            self.armPeriodView.setText('(Max ???')


        if (self.model.latest_bac_countdown is not None and time.time() - self.model.last_heard_arm_countdown < self.model.WATCHDOG):
            self.armLockoutView.setText('%f s left' % self.model.latest_bac_countdown)
        else:
            self.armLockoutView.setText("??? s left")

        if (self.model.last_est_accel and self.model.last_heard_fsm_state_high_rate - time.time() < self.model.WATCHDOG):
            self.fsmAccView.addDataPoint(0, time.time(), self.model.last_est_accel)

STATE_CODE_TO_NAME = {
        0:'ESTOP', # All actuator power off
        1:'SHUTDOWN', # Gently stops system
        2:'SOFT_STOP', # If brakes are not in auto mode, same as ESTOP. Else, commands intermediate setpoint
        10:'ARMING',  # Gets the system into ARM by switching to AUTO, then opening brakes against hardstops.
        11:'ARM',     # System watches FC IMU info and transitions to LAUNCH when sufficient forward accel seen.
        12:'LAUNCH',  # Transitioned to from ARM after sufficient forward accel. Watchs FC IMU to transition to FLIGHT
        13:'FLIGHT',  # Launch seems to have ended. Broadcasts that we're in flight, lets brake controller do its thing.
        20:'PRE_DRIVE', # Prepares drive mode
        21:'DRIVE', #Accepts drive teleoperation modes
        99:'TELEOP'
      }

class ModeControlUI(QtGui.QWidget):
    def __init__(self, lc, name=None, parent=None):
        super(QtGui.QWidget, self).__init__(parent)
        self.lc = lc
        if name:
            self.setObjectName(name)

        # quick-check the integrity of our STATE_CODES table:
        print STATE_CODE_TO_NAME
        for code in STATE_CODE_TO_NAME:
            valid = hasattr(state_t, STATE_CODE_TO_NAME[code])
            assert (hasattr(state_t, STATE_CODE_TO_NAME[code]) and getattr(state_t, STATE_CODE_TO_NAME[code]) == code), \
                'ERROR: Inconsistency between lcmtype enums and UI code-state pair: (' + str(code) 
        self.model = ModeUIModel(lc, STATE_CODE_TO_NAME)
        self.view = ModeUIView(lc, self.model, STATE_CODE_TO_NAME)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.view)
        self.setLayout(layout)


if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    overallWidget = QtGui.QWidget()
    modecontrol = ModeControlUI(lc)
    overallLayout = QtGui.QVBoxLayout(overallWidget)
    overallLayout.addWidget(modecontrol)
    overallWidget.setWindowTitle('Mode Control Buttons')
    overallWidget.show()
    overallWidget.resize(600,300)

    # Test Handlers with special values
    init_state = state_t()
    init_state.currentState = -1
    modecontrol.model.new_state_handler('FSM_STATE',init_state.encode())
    init_config = bac_config_t()
    init_config.minimum_disarm_duration = -1
    modecontrol.model.new_config_handler('_BAC_CONFIG_STATE',init_config.encode())

    start_lcm(lc)
    
    sys.exit(app.exec_())
