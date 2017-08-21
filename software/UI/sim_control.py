#!/usr/bin/env python

#signif reference to http://pastebin.com/k87sfiEf


import sys
import math
import signal
import time 
import os

import math, random
import numpy as np

#interface stuff
from PyQt4 import QtCore, QtGui, QtOpenGL
import pyqtgraph as pg

# plotter
from data_plot_widget import DataPlotWidget

#comms stuff
import lcm
from mithl import trigger_t
from mithl import sim_pusher_cmd_t
from mithl import floating_base_t
from lcm_utils import *

#read yaml config information
import yaml


class GroundTruthVisWidget(QtGui.QWidget):
    ''' Plots pod linear motion in tube, reading
    ground truth outputs from sim.'''
    def __init__(self, config, lc=None, parent=None, name=None,):
        super(GroundTruthVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()


        self.positionPlotter = DataPlotWidget(2, title="Position", dataRange=[0, 100], alsoNumeric=True)
        self.velocityPlotter = DataPlotWidget(1, title="Velocity", dataRange=[-20, 20], alsoNumeric=True)

        plotLayout = QtGui.QVBoxLayout()    
        plotLayout.addWidget(self.positionPlotter)
        plotLayout.addWidget(self.velocityPlotter)
        self.setLayout(plotLayout)

        self.last_fb = None
        floatingBaseSub = self.lc.subscribe("SIM_FB", self.handleSimFloatingBase)

    def handleSimFloatingBase(self, channel, data):
        msg = floating_base_t.decode(data)
        t = msg.utime / 1000. / 1000.
        if (self.last_fb == None or t - self.last_fb >= 0.05):
            self.positionPlotter.addDataPoint(0, t, msg.q[0])
            self.velocityPlotter.addDataPoint(0, t, msg.v[0])
            self.last_fb = t

class PusherControllerWidget(QtGui.QWidget):
    ''' Plots pod linear motion in tube, reading
    ground truth outputs from sim.'''
    def __init__(self, config, lc=None, parent=None, name=None,):
        super(PusherControllerWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        mainLayout = QtGui.QVBoxLayout()

        forceLayout = QtGui.QHBoxLayout()
        forceLabel = QtGui.QLabel('Force:')
        self.forceSpinbox = QtGui.QDoubleSpinBox()
        self.forceSpinbox.setRange(0., 1000000.)
        self.forceSpinbox.setValue(1000.)
        self.forceSpinbox.setSuffix('N')
        forceLayout.addWidget(forceLabel)
        forceLayout.addWidget(self.forceSpinbox)
        mainLayout.addLayout(forceLayout)

        durationLayout = QtGui.QHBoxLayout()
        durationLabel = QtGui.QLabel('Duration:')
        self.durationSpinbox = QtGui.QDoubleSpinBox()
        self.durationSpinbox.setRange(0., 50.)
        self.durationSpinbox.setValue(2.)
        self.durationSpinbox.setSuffix('s')
        durationLayout.addWidget(durationLabel)
        durationLayout.addWidget(self.durationSpinbox)
        mainLayout.addLayout(durationLayout)

        self.pusherStartButton = QtGui.QPushButton('Start Pusher', self)
        self.pusherStartButton.clicked.connect(self.handlePusherStartButton)
        mainLayout.addWidget(self.pusherStartButton)

        self.setLayout(mainLayout)

    def handlePusherStartButton(self):
        sendTime = time.time()
        pusher_start = sim_pusher_cmd_t()
        pusher_start.utime = int((sendTime - self.startTime)* 1000000)
        pusher_start.force = self.forceSpinbox.value()
        pusher_start.duration = self.durationSpinbox.value()
        self.lc.publish("SIM_PUSHER_CMD",pusher_start.encode())


class SimulationControlWidget(QtGui.QWidget):
    ''' Controls simulation. Bare-bones right now '''
    def __init__(self, config, lc=None, parent=None, name=None):
        super(SimulationControlWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()
        
        mainLayout = QtGui.QVBoxLayout()
        
        groundTruthVis = GroundTruthVisWidget(config, lc=lc)
        mainLayout.addWidget(groundTruthVis)
        
        pusherController = PusherControllerWidget(config, lc=lc)
        mainLayout.addWidget(pusherController)
        
        self.resetButton = QtGui.QPushButton('Reset', self)
        self.resetButton.clicked.connect(self.handleResetButton)
        
        buttonLayout = QtGui.QHBoxLayout()
        buttonLayout.addWidget(self.resetButton)

        mainLayout.addLayout(buttonLayout)     
        self.setLayout(mainLayout)

    def update(self):
        pass
        

    def handleResetButton(self):
        sendTime = time.time()
        reset_msg = trigger_t()
        reset_msg.utime = int((sendTime - self.startTime)* 1000000)
        self.lc.publish("SIM_RESET",reset_msg.encode())

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)


    with open(os.environ['MIT_HL_ROOT'] + '/config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = SimulationControlWidget(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())