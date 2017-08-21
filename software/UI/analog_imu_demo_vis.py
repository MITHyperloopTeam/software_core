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
from lcm_utils import *

#read yaml config information
import yaml

from pod_pyopengl import PodGLWidget
from analog_vis import AnalogPlotWidget


class Window(QtGui.QWidget):
    ''' The entire UI window. Everything lives in here.'''
    def __init__(self, config, vis_config, vis_config_file_path, lc=None):
        super(Window, self).__init__()

        self.lc = lc
        self.glWidget = PodGLWidget(vis_config=vis_config, vis_config_file_path=vis_config_file_path)
        self.glWidget.lineOffset = 5000
        self.analogVis1 = AnalogPlotWidget(indices=[3], title="Linear Potentiometer", channel="VEC", dataRange=[0, 1200], color=[0, 0, 255], histLength=100, lc=lc)
        self.analogVis2 = AnalogPlotWidget(indices=[8], title="Temperature #1", channel="VEC", dataRange=[0, 120], color=[255, 0, 0], histLength=100, lc=lc)
        #self.analogVis3 = AnalogPlotWidget(indices=[8], title="Temperature #2", channel="VEC", dataRange=[0, 120], color=[255, 0, 0], histLength=100, lc=lc)

        mainLayout = QtGui.QGridLayout()

        mainLayout.addWidget(self.glWidget, 1, 0, 4, 6)
        mainLayout.addWidget(self.analogVis1, 0, 6, 3, 1)
        mainLayout.addWidget(self.analogVis2, 3, 6, 3, 1)
        #mainLayout.addWidget(self.analogVis3, 4, 6, 2, 1)

        self.setLayout(mainLayout)

        self.setWindowTitle("ANALOG + IMU DEMO")

        self.lc.subscribe("IMU", self.handle_imu_t)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        self.glWidget.get_camera().yaw += 0.1

    def handle_imu_t(self, channel, data):
        msg = vectorXf_t.decode(data)
        offset = [None, None, None, msg.data[1], -msg.data[0],-msg.data[2]]
        self.glWidget.set_offset(None, offset)
        self.glWidget.update()

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)

    vis_config_file = "../models/pod.yaml"
    vis_config_file_path = os.path.abspath(os.path.dirname(vis_config_file))

    with open(vis_config_file, 'r') as f:
        vis_config = yaml.load(f)
    
    lc = create_lcm()

    app     = QtGui.QApplication(sys.argv)
    mainWindow = Window(config, vis_config, vis_config_file_path, lc)

    try:
        mainWindow.show()
    except KeyboardInterrupt:
        print "Interrupted"

    start_lcm(lc)

    sys.exit(app.exec_())
