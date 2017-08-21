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
from mithl import velocity_t
from mithl import auto_braking_t
from lcm_utils import *

#read yaml config information
import yaml

class BrakingWidget(QtGui.QWidget):
    ''' Pod Visualization window. Plots pod state with pyqtgraph and an OpenGL window.
        Room for more stuff here.'''
    def __init__(self, config, lc=None, parent=None, name=None):
        super(BrakingWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        brakingLayout = QtGui.QVBoxLayout()

        self.brakingSlider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.brakingSlider.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.brakingSlider.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.brakingSlider.setTickInterval(1)
        self.brakingSlider.setSingleStep(1)
        self.brakingSlider.setRange(1,25)
        self.brakingSlider.setValue(17)
        
        self.brakingSlider.valueChanged.connect(self.handle_braking_slider)

        self.brakingLabel = QtGui.QLabel("Braking")
        self.brakingLabel.setAlignment(QtCore.Qt.AlignCenter)
        
        brakingLayout.addWidget(self.brakingLabel)
        brakingLayout.addWidget(self.brakingSlider)
                     
        self.setLayout(brakingLayout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

        self.handle_braking_slider()

    def update(self):
        pass
        
    def handle_braking_slider(self): #higher braking slope means more braking aggression
        auto_braking_msg = auto_braking_t()
        auto_braking_msg.slope = -self.brakingSlider.value()/100.0
        auto_braking_msg.desiredDistanceToEnd = 50.0
        auto_braking_msg.kP = 100.0
        print "Requested braking slope: "+str(auto_braking_msg.slope)
        self.lc.publish("AUTO_BRAKING", auto_braking_msg.encode())


if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
        
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = BrakingWidget(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
