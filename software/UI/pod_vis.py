#!/usr/bin/env python

#signif reference to http://pastebin.com/k87sfiEf

import sys
import math
import signal
import time 
import os
from threading import Lock

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
from mithl import auto_braking_t
from mithl import velocity_t
from mithl import floating_base_t
from lcm_utils import *

#read yaml config information
import yaml

from pod_pyopengl import PodGLWidget

class PodVisWidget(QtGui.QWidget):
    ''' Pod Visualization window. Plots pod state with pyqtgraph and an OpenGL window.
        Room for more stuff here.'''
    def __init__(self, config, lc=None, parent=None, name=None, vis_config=None, vis_config_file_path=None):
        super(PodVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        self.glWidget = PodGLWidget(vis_config=vis_config, vis_config_file_path=vis_config_file_path)
        self.vis_config = vis_config

        self.plot = pg.PlotWidget(title="Position in tube")
        self.plot.setYRange(0,config['tube']['length'],padding=0.1)
        self.curves = []
        for i in range(11):
            self.curves.append(self.plot.plot([0], [0], pen=(i, 11)))

        self.resetDataBuf()
            
        #self.setWindowTitle("BrakingSliders")
        
        mainLayout = QtGui.QVBoxLayout()
        mainLayout.addWidget(self.glWidget)
        mainLayout.addWidget(self.plot)
        self.setLayout(mainLayout)

        self.plotterMutex = Lock()
        podchannel_sub = self.lc.subscribe("SIM_FB", self.handle_vectorXf_t)
        #podchannel_sub = self.lc.subscribe("_SE_FB", self.handle_vectorXf_t)   
        auto_braking_sub = self.lc.subscribe("_BC", self.handle_auto_braking_t)
        ls_sub = self.lc.subscribe("TELEOP_VELOCITY", self.handle_ls_command)

        imu_sub = lc.subscribe("IMU", self.handle_imu_t)
        imu_sub = lc.subscribe("SIM_IMU", self.handle_sim_imu_t)

        podse_sub = self.lc.subscribe("_SE_FB", self.handle_floating_base_t)

        self.lastSwitchUpDown = time.time()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def resetDataBuf(self):
        self.ptr = 0
        self.histLength = 1000
        self.good_ptr = 0

        self.podDataHist = []
        self.timeDataHist = np.zeros(self.histLength)
        for i in range(11):
            self.podDataHist.append(np.zeros(self.histLength))

    def update(self):
        self.plotterMutex.acquire()
        try:
            for i in range(11):
                self.curves[i].setData(self.timeDataHist[0:self.good_ptr], self.podDataHist[i][0:self.good_ptr])
                #self.plot.setXRange(np.min(self.timeDataHist[0:self.good_ptr], np.max(self.timeDataHist[0:self.good_ptr])))
                #self.plot.setXRange(0, 1000)
        except Exception as e:
            print e
        self.plotterMutex.release()

        self.glWidget.get_camera().yaw += 0.5
        if (time.time() - self.lastSwitchUpDown > 30.):
            self.lastSwitchUpDown = time.time()
            if (self.glWidget.get_camera().pitch < 0):
                self.glWidget.get_camera().pitch = 30
            else:
                self.glWidget.get_camera().pitch = -1
        self.glWidget.update()

    def handle_vectorXf_t(self, channel, data):
        self.plotterMutex.acquire()
        try:
            msg = vectorXf_t.decode(data)
            if (self.ptr > 0 and self.timeDataHist[self.ptr-1] > float(msg.utime)/1000000):
                self.resetDataBuf()
            
            self.timeDataHist[self.ptr] = float(msg.utime)/1000000
            
            for i in range(11):
                if (i < msg.rows):
                    if self.ptr == self.histLength-1 and self.good_ptr == self.histLength-1:
                        # todo: this is horrendously inefficient
                        self.podDataHist[i] = np.roll(self.podDataHist[i], -1)
                    self.podDataHist[i][self.ptr] = msg.data[i]

            if self.ptr == self.histLength-1 and self.good_ptr == self.histLength-1:
                self.timeDataHist = np.roll(self.timeDataHist, -1)

            self.good_ptr = max(self.ptr, self.good_ptr)
            self.ptr = min(self.ptr+1,  self.histLength-1)
        except Exception as e:
            print e
        self.plotterMutex.release()

    def handle_auto_braking_t(self, channel, data):
        msg = vectorXf_t.decode(data)
        brake_default_color = self.glWidget.get_default_color("Brakes")
        braking_color = [1.0, 0.0, 0.0, 0.5]
        des_color = [msg.data[0]/8000.*braking_color[i] + (1.0-msg.data[0]/8000)*brake_default_color[i] for i in range(3)]
        self.glWidget.set_color("Brakes", des_color)
        self.glWidget.update()


    def handle_ls_command(self, channel, data):
        msg = velocity_t.decode(data)
        vel = msg.desiredVelocity
        self.glWidget.set_color("LowSpeed", [max(0.0, min(1.0, 1.0-abs(msg.desiredVelocity))), 
                                             max(0.0, min(1.0, 1.0+msg.desiredVelocity)), 
                                             max(0.0, min(1.0, 1.0-msg.desiredVelocity)), 
                                             1.0])
        self.glWidget.update()

    def handle_imu_t(self, channel, data):
        msg = vectorXf_t.decode(data)
        offset = [None, None, None, msg.data[1], -msg.data[0],-msg.data[2]]
        self.glWidget.set_offset(None, offset)
        self.glWidget.update()

    def handle_sim_imu_t(self, channel, data):
        msg = vectorXf_t.decode(data)
        offset = [None, None, self.glWidget.global_offset[3]*0.9 + msg.data[0]*0.1, None, None, None]
        self.glWidget.set_offset(None, offset)
        self.glWidget.update()

    def handle_floating_base_t(self, channel, data):
        msg = floating_base_t.decode(data)
        self.glWidget.lineOffset = msg.q[0]*100.

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

    app = QtGui.QApplication(sys.argv)
    window = PodVisWidget(config, lc=lc, vis_config=vis_config, vis_config_file_path=vis_config_file_path)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
