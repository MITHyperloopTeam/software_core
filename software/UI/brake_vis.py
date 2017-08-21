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
from mithl import bac_state_high_rate_t
from lcm_utils import *

#read yaml config information
import yaml

class BrakeVisWidget(QtGui.QWidget):
    ''' Brake end-on visualization window. Draws brake state manually for basic visualization.'''
    def __init__(self, config, lc=None, parent=None, name=None):
        super(BrakeVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()
        self.config = config

        self.setMinimumSize(QtCore.QSize(100, 100))
        self.grview = QtGui.QGraphicsView()
        self.grscene = QtGui.QGraphicsScene()
        self.grview.setScene(self.grscene)
        # scene units are in mm

        imgLeft = QtGui.QImage("../models/brake_left_caliper_only.png")
        imgLeft = imgLeft.convertToFormat(QtGui.QImage.Format_ARGB32_Premultiplied)
        #imgLeft = imgLeft.rgbSwapped()
        
        imgRight = QtGui.QImage("../models/brake_right_caliper_only.png")
        imgRight = imgRight.convertToFormat(QtGui.QImage.Format_ARGB32_Premultiplied)
       # imgRight = imgRight.rgbSwapped()

        imgRail = QtGui.QImage("../models/brake_rail_only.png")
        imgRail = imgRail.convertToFormat(QtGui.QImage.Format_ARGB32_Premultiplied)
        imgRail = imgRail.rgbSwapped()
        
        self.PX_PER_INCH = 52.4
        self.SCALING = 0.75
        self.total_inch_to_im_scale = self.PX_PER_INCH * self.SCALING 
        # we're going to render in inches, so scale image approp

        
        pxLeft = QtGui.QPixmap()
        pxLeft.convertFromImage(imgLeft)
        self.caliperLeftFront = QtGui.QGraphicsPixmapItem(pxLeft)
        self.caliperLeftBack = QtGui.QGraphicsPixmapItem(pxLeft)
        self.caliperLeftBack.setOpacity(0.5)
        self.caliperLeftFront.setOpacity(1.0)

        pxRight = QtGui.QPixmap()
        pxRight.convertFromImage(imgRight)
        self.caliperRightFront = QtGui.QGraphicsPixmapItem(pxRight)
        self.caliperRightBack = QtGui.QGraphicsPixmapItem(pxRight)
        self.caliperRightBack.setOpacity(0.5)
        self.caliperRightFront.setOpacity(1.0)

        pxRail = QtGui.QPixmap()
        pxRail.convertFromImage(imgRail)
        self.rail = QtGui.QGraphicsPixmapItem(pxRail)


        self.caliperRightFront.scale(self.SCALING, self.SCALING)
        self.caliperRightBack.scale(self.SCALING, self.SCALING)
        self.caliperLeftFront.scale(self.SCALING, self.SCALING)
        self.caliperLeftBack.scale(self.SCALING, self.SCALING)
        self.rail.scale(self.SCALING, self.SCALING)

        self.grscene.addItem(self.rail)
        self.grscene.addItem(self.caliperRightBack)
        self.grscene.addItem(self.caliperLeftBack)
        self.grscene.addItem(self.caliperRightFront)
        self.grscene.addItem(self.caliperLeftFront)

        # home position is FULLY EXTENDED POT
        self.caliperLeft_HomePos = -2.5*self.total_inch_to_im_scale
        self.caliperLeft_Pos = 0
        self.caliperRight_HomePos = 2.5*self.total_inch_to_im_scale
        self.caliperRight_Pos = 0

        self.caliperVert_HomePos = -0.25*self.total_inch_to_im_scale
        self.caliperVert_Pos_Front = 0
        self.caliperVert_Pos_Back = 0

        # add a nice gradient background
        gradient = QtGui.QLinearGradient(-100, -100, 100, 100)
        gradient.setColorAt(-1, QtGui.QColor(20, 60, 20))
        gradient.setColorAt(1, QtGui.QColor(80, 80, 120))
        self.grscene.setBackgroundBrush(QtGui.QBrush(gradient))

        mainLayout = QtGui.QVBoxLayout()
        mainLayout.addWidget(self.grview)
        self.setLayout(mainLayout)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

        podse_sub = self.lc.subscribe("_BAC_STATE_H", self.handle_bac_state_high_rate_t)

    def update(self):
        self.caliperLeftFront.setX(self.caliperLeft_HomePos - self.caliperLeft_Pos)
        self.caliperLeftBack.setX(self.caliperLeft_HomePos - self.caliperLeft_Pos)
        self.caliperRightFront.setX(self.caliperRight_HomePos + self.caliperRight_Pos)
        self.caliperRightBack.setX(self.caliperRight_HomePos + self.caliperRight_Pos)

        self.caliperLeftFront.setY(self.caliperVert_HomePos + self.caliperVert_Pos_Front)
        self.caliperLeftBack.setY(self.caliperVert_HomePos + self.caliperVert_Pos_Back)
        self.caliperRightFront.setY(self.caliperVert_HomePos + self.caliperVert_Pos_Front)
        self.caliperRightBack.setY(self.caliperVert_HomePos + self.caliperVert_Pos_Back)

    def handle_bac_state_high_rate_t(self, channel, data):
        msg = bac_state_high_rate_t.decode(data)
        d1 = msg.distance_1
        d2 = msg.distance_2
        gh_front = msg.gh_front
        gh_rear = msg.gh_rear

        # apply a little low pass
        # flip pots (length 5.9, so subtract from that) to get distance inward the caliper has traveled from home
        self.caliperLeft_Pos =  self.caliperLeft_Pos*0.5 + 0.5*-1.0*(5.9-d2) * self.total_inch_to_im_scale
        self.caliperRight_Pos = self.caliperRight_Pos*0.5 + 0.5*-1.0*(5.9-d1) * self.total_inch_to_im_scale

        # units are mm, and hardcode "resting on rail" from logs:
        # 38 mm for front
        # 39 mm for rear
        # higher = farther down
        self.caliperVert_Pos_Front = ((gh_front - 38)/25.4)*self.total_inch_to_im_scale 
        self.caliperVert_Pos_Rear = ((gh_rear - 39)/25.4)*self.total_inch_to_im_scale

        #  d1 + d2 = 2.35in when closed
        #  d1 + d2 = 7.65in when open
        # || -------> |    || d1
        # ||   |******|
        # ||   |<----------|| d2


if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = BrakeVisWidget(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
