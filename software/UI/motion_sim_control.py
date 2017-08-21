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
from lcm_utils import *

#read yaml config information
import yaml

class MotionSimControl(QtGui.QWidget):
    def __init__(self, config, lc=None, parent=None, name=None):
        super(MotionSimControl, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        self.PitchDial = QtGui.QDial()
        self.PitchDial.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.PitchDial.setNotchesVisible(True)
        self.maxSpeed = 90.0    
        self.PitchDial.setRange(-self.maxSpeed,self.maxSpeed) #rad/s
        
        self.PitchLabel = QtGui.QLabel("Pitch Desired")
        self.PitchLabel.setAlignment(QtCore.Qt.AlignCenter)
        
        velocityLayout = QtGui.QVBoxLayout()
        velocityLayout.addWidget(self.PitchLabel)
        velocityLayout.addWidget(self.PitchDial)

        self.RollDial = QtGui.QDial()
        self.RollDial.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.RollDial.setNotchesVisible(True)
        self.maxSpeed = 90.0    
        self.RollDial.setRange(-self.maxSpeed,self.maxSpeed) #rad/s

        self.RollLabel = QtGui.QLabel("Roll Desired")
        self.RollLabel.setAlignment(QtCore.Qt.AlignCenter)
        
        velocityLayout.addWidget(self.RollLabel)
        velocityLayout.addWidget(self.RollDial)


        teleopControlLayout = QtGui.QHBoxLayout()
        #teleopControlLayout.addLayout(brakingLayout)
        teleopControlLayout.addLayout(velocityLayout)
        
        teleopLayout = QtGui.QVBoxLayout()
        teleopLayout.addLayout(teleopControlLayout)

        #self.setWindowTitle("BrakingSliders")
             
        self.setLayout(teleopLayout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        cmd = vectorXf_t()
        cmd.rows = 2
        cmd.data = [self.PitchDial.value(), self.RollDial.value()]
        self.lc.publish("MOTION_SIM_CMD", cmd.encode())


if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = MotionSimControl(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
