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
from mithl import string_t
from mithl import state_t
from lcm_utils import *

#read yaml config information
import yaml

try:
    from OpenGL import GL
except ImportError:
    app = QtGui.QApplication(sys.argv)
    QtGui.QMessageBox.critical(None, "PyQT with OpenGL",
            "PyOpenGL must be installed to run this code.")
    sys.exit(1)

class PodDriverWidget(QtGui.QWidget):
    ''' Pod Visualization window. Plots pod state with pyqtgraph and an OpenGL window.
        Room for more stuff here.'''
    def __init__(self, config, lc=None, parent=None, name=None):
        super(PodDriverWidget, self).__init__(parent)

        self.startTime = time.time()

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        self.currentState = ''
        self.stateLabel = QtGui.QLabel(self.currentState)
        
        temp_state = state_t()
                
        self.arm = int(temp_state.ARM)
        self.launch = int(temp_state.LAUNCH)
        self.flight = int(temp_state.FLIGHT)
        self.safe = int(temp_state.SAFE)
        self.floating = int(temp_state.FLOATING)
        self.drive = int(temp_state.DRIVE)
        self.soft_stop = int(temp_state.SOFT_STOP)
        self.estop = int(temp_state.ESTOP)
        self.fault = int(temp_state.FAULT)

        self.critical_states = [self.estop, self.soft_stop, self.fault]
        self.caution_states = [self.arm, self.launch, self.flight, self.floating, self.drive]
        self.safe_states = [self.safe]

        self.state_lst = [self.arm,self.launch,self.flight,self.safe,self.floating, self.drive, self.soft_stop,self.estop,self.fault]
        self.state_string_lst = ['ARM', 'LAUNCH', 'FLIGHT', 'SAFE', 'FLOATING', 'DRIVE', 'SOFT_STOP', 'ESTOP', 'FAULT']
        self.state_dict = dict(zip(self.state_string_lst, self.state_lst))
        self.reverse_state_dict = dict(zip(self.state_lst, self.state_string_lst))

        self.stateDropdown = QtGui.QComboBox(self)
        for state in self.state_string_lst:
            self.stateDropdown.addItem(state)
            
        self.stateDropdown.activated[str].connect(self.publish_state_request)
        
        
        self.softStopButton = QtGui.QPushButton('SOFT-STOP', self)
        self.softStopButton.setStyleSheet('background-color: rgba(255, 255, 0, 150); font-size: 18pt;')
        self.softStopButton.clicked.connect(self.publish_softstop_request)
        self.softStopButton.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding))

        self.eStopButton = QtGui.QPushButton('E-STOP', self)
        self.eStopButton.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding))
        self.eStopButton.setStyleSheet('background-color: rgba(255, 0, 0, 150); font-size: 18pt;')
        self.eStopButton.clicked.connect(self.publish_estop_request)


        mainLayout = QtGui.QGridLayout()
        mainLayout.addWidget(self.softStopButton, 0, 0, 2, 1)
        mainLayout.addWidget(self.eStopButton, 0, 1, 2, 1)
        mainLayout.addWidget(self.stateLabel, 2, 0, 1, 1)
        mainLayout.addWidget(self.stateDropdown, 2, 1, 1, 1)

        self.setLayout(mainLayout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

        currentstate_sub = self.lc.subscribe("STATE", self.handle_state_t)

    def publish_softstop_request(self):
        self.publish_state_request("SOFT_STOP")

    def publish_estop_request(self):
        self.publish_state_request("ESTOP")

    def publish_state_request(self, requested_state):
        sendTime = time.time()
        state_msg = state_t()
        state_msg.utime = int((sendTime - self.startTime)* 1000000)
        state_msg.currentState = self.state_dict[str(requested_state)]
        self.lc.publish("REQUESTED_STATE",state_msg.encode())

    def handle_state_t(self, channel, data):
        msg = state_t.decode(data)
        self.currentState = int(msg.currentState)

    def update(self):
        if self.currentState in self.reverse_state_dict.keys():
            self.stateLabel.setText("CURRENT STATE: " + self.reverse_state_dict[self.currentState])
            if self.currentState in self.safe_states:
                self.stateLabel.setStyleSheet('background-color: None')
            elif self.currentState in self.caution_states:
                self.stateLabel.setStyleSheet('background-color: rgba(255, 255, 0, 100);')
            else:
                self.stateLabel.setStyleSheet('background-color: rgba(255, 0, 0, 100);')
        else:
            self.stateLabel.setText("NONE")
            self.stateLabel.setStyleSheet('background-color: rgba(255, 0, 0, 100);')

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = PodDriverWidget(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
