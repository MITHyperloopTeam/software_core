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
from mithl import floating_base_t
from mithl import particle_filter_t
from mithl import vectorXf_t
from mithl import state_estimator_particle_set
from mithl import state_estimator_particle
from lcm_utils import *

#read yaml config information
import yaml

class PodTubeVisWidget(QtGui.QWidget):
    ''' Pod Visualization window. Plots pod state with pyqtgraph.'''
    def __init__(self, config, lc=None, parent=None, name=None):
        super(PodTubeVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()
        self.config = config

        self.setMinimumHeight(200)

        self.plot = pg.PlotWidget(title="State Estimation")
        self.plot.setXRange(0,float(config['tube']['length']),padding=0.1)
        self.plot.hideAxis("left")

        img = QtGui.QImage("../models/pod.png")
        img = img.convertToFormat(QtGui.QImage.Format_ARGB32_Premultiplied)
        img = img.rgbSwapped()
        img = img.mirrored(False, True)

        imgArray = np.float64(pg.imageToArray(img, copy=True))
        self.img_mle = pg.ImageItem(imgArray, opacity=0.0)
        self.img_gt = pg.ImageItem(imgArray, opacity=0.9)
        self.img_aspect = float(imgArray.shape[1]) / float(imgArray.shape[0])
        self.pod_mle = 0.0
        self.pod_gt = 0.0

        self.viewBox = self.plot.getViewBox()
        self.viewBox.setMouseEnabled(x=True, y=False)
        self.viewBox.setYRange(-0.5, 0.5)
        self.viewBox.setBackgroundColor([50, 80, 80])

        # add a nice gradient background
        self.gradBackground = QtGui.QGraphicsRectItem(0, -1, config["tube"]["length"], 2)
        gradient = QtGui.QLinearGradient(0, -1, 0, 2)
        gradient.setColorAt(0.0, QtGui.QColor(50, 50, 50))
        gradient.setColorAt(1.0, QtGui.QColor(40, 40, 160))
        self.gradBackground.setBrush(QtGui.QBrush(gradient))
        self.viewBox.addItem(self.gradBackground)

        # add the fiducial markers at half opacity
        line_center = config["tube"]["length"] - config["tube"]["distance_after_last_fiducial"]
        self.lines = []
        self.lineColor = QtGui.QColor(200, 200, 0)
        self.lineWidth = config["tube"]["fiducial_width"]
        while (line_center > 0):
            line = QtGui.QGraphicsLineItem(line_center, -1.0, line_center, 1.0)
            self.lines.append(line)
            self.viewBox.addItem(line)
            line_center -= config["tube"]["fiducial_separation"]

        # add the keep-outs and back and front
        backZone = QtGui.QGraphicsRectItem(-50000, -1, 50000, 2)
        #backZone.setPen(QtCore.Qt.NoPen)
        backZone.setBrush(QtGui.QBrush(QtGui.QColor(200, 50, 50), QtCore.Qt.Dense1Pattern))
        self.viewBox.addItem(backZone)
        frontZone = QtGui.QGraphicsRectItem(config["tube"]["length"], -1, 50000, 2)
        #backZone.setPen(QtCore.Qt.NoPen)
        frontZone.setBrush(QtGui.QBrush(QtGui.QColor(200, 50, 50), QtCore.Qt.Dense1Pattern))
        self.viewBox.addItem(frontZone)

        self.particles = np.zeros((0, 3))
        self.particles_scatter = pg.ScatterPlotItem()
        self.viewBox.addItem(self.particles_scatter)

        self.viewBox.addItem(self.img_mle)
        self.viewBox.addItem(self.img_gt)


        self.densityCurve = self.plot.plot([0], [0], pen=pg.mkPen([255, 0, 0]))

        #self.setWindowTitle("BrakingSliders")
        
        mainLayout = QtGui.QVBoxLayout()
        mainLayout.addWidget(self.plot)
        self.setLayout(mainLayout)

        self.podse_sub = self.lc.subscribe("_FC_SE", self.handle_state_estimate)
        self.podfb_sub = self.lc.subscribe("SIM_FB", self.handle_ground_truth)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        viewXSize = float(self.viewBox.viewRange()[0][1] - self.viewBox.viewRange()[0][0])
        viewYSize = float(self.viewBox.viewRange()[1][1] - self.viewBox.viewRange()[1][0])

        # given MLE, make view range track pod
        xmin, xmax = self.viewBox.viewRange()[0]
        borderRatio = 0.4
        softxmax = borderRatio*xmin + (1.0-borderRatio)*xmax
        softxmin = (1.0-borderRatio)*xmin + borderRatio*xmax
        if (self.pod_gt - softxmax >= 0):
            xmin += (self.pod_gt - softxmax)*0.25
            xmax += (self.pod_gt - softxmax)*0.25
        elif (self.pod_gt - softxmin <= 0):
            xmin += (self.pod_gt - softxmin)*0.25
            xmax += (self.pod_gt - softxmin)*0.25
        self.viewBox.setRange(xRange=(xmin, xmax), padding=0.0)

        # might need to generate these
        viewXSize = xmax - xmin

        minScale = 1./10.0
        # draw as either 2.0 meters long, or minScale total line length, preserving
        # aspect ratio in view pixel
        actualViewRatio = self.viewBox.viewPixelSize() # size of one screen pixel in view coords
        viewRatioAdjustment = float(actualViewRatio[0]) / float(actualViewRatio[1])
        mleDrawLength = max(2.0, viewXSize * minScale)
        mleDrawHeight = mleDrawLength * self.img_aspect / viewRatioAdjustment
        mleDrawX = self.pod_mle - mleDrawLength / 2.
        mleDrawY = viewYSize/10.0 + self.viewBox.viewRange()[1][0] - mleDrawHeight / 2.0
        mleDrawRect = QtCore.QRectF(mleDrawX, mleDrawY, mleDrawLength, mleDrawHeight)
        self.img_mle.setRect(mleDrawRect)

        gtDrawX = self.pod_gt - mleDrawLength / 2.
        gtDrawRect = QtCore.QRectF(gtDrawX, mleDrawY, mleDrawLength, mleDrawHeight)
        self.img_gt.setRect(gtDrawRect)

        for line in self.lines:
            # lines must be at least 1 px wide
            line.setPen(QtGui.QPen(self.lineColor, max(self.lineWidth, actualViewRatio[0]) , QtCore.Qt.SolidLine))



        if len(self.particles) > 0:
            weights = np.array([p[2] for p in self.particles])
            normalized_weights = weights / np.max(weights)
            self.particles_scatter.setData(np.array([p[0][0] for p in self.particles]), 0.5*normalized_weights-0.25, pen=pg.mkPen([0, 125, 255, 150], width=5))

            # build up sample points
            densityX = np.array([xmin, xmax])
            for p in self.particles:
                densityX = np.append(densityX, np.arange(p[0][0]-p[1][0][0]*4, p[0][0]+p[1][0][0]*4, max(p[1][0][0]/2, 0.01)))

            densityX = np.sort(densityX)
            densityY = np.zeros(densityX.shape)
            for p in self.particles:
                densityY += p[2] * np.exp( - (densityX - p[0][0])**2 / p[1][0][0]**2) / np.sqrt(2 * math.pi * max(p[1][0][0]/2, 0.01)**2)


            densityY /= np.max(densityY)*1.5
            densityY -= -mleDrawY 

            self.densityCurve.setData(densityX, densityY)


    def handle_state_estimate(self, channel, data):
        msg = state_estimator_particle_set.decode(data)
        self.pod_mle = msg.particles[0].mu[0]

        particles = []
        for i in range(msg.n_particles):
            if msg.particles[i].id >= 0 and msg.particles[i].weight > 0.:
                particles.append([msg.particles[i].mu,
                                  msg.particles[i].Sigma,
                                  msg.particles[i].weight])
        self.particles = particles
        self.pod_gt = msg.particles[0].mu[0]
    
    def handle_ground_truth(self, channel, data):
        msg = floating_base_t.decode(data)
        #self.pod_gt = msg.q[0]


if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = PodTubeVisWidget(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
