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
from mithl import analog_front_medium_rate_t, analog_rear_medium_rate_t
from lcm_utils import *

#read yaml config information
import yaml

class LCMVisWidget(QtGui.QWidget):
    '''LCM end-on visualization window. Draws LCM state manually for basic visualization.'''
    def __init__(self, config, lc=None, parent=None, name=None):
        super(LCMVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()
        self.config = config

        self.setMinimumSize(QtCore.QSize(200, 200))
        self.grview = QtGui.QGraphicsView()
        self.grscene = QtGui.QGraphicsScene()
        self.grview.setScene(self.grscene)
        # scene units are in mm

        imgLCM = QtGui.QImage("../models/lcm_only.png")
        imgLCM = imgLCM.convertToFormat(QtGui.QImage.Format_ARGB32_Premultiplied)

        imgRail = QtGui.QImage("../models/lcm_rail_only.png")
        imgRail = imgRail.convertToFormat(QtGui.QImage.Format_ARGB32_Premultiplied)
        imgRail = imgRail.rgbSwapped()
        
        self.PX_PER_INCH = 57
        self.SCALING = 0.5
        self.total_inch_to_im_scale = self.PX_PER_INCH * self.SCALING 
        # we're going to render in inches, so scale image approp

        
        pxLCM = QtGui.QPixmap()
        pxLCM.convertFromImage(imgLCM)
        self.LCMFront = QtGui.QGraphicsPixmapItem(pxLCM)
        self.LCMBack = QtGui.QGraphicsPixmapItem(pxLCM)
        self.LCMBack.setOpacity(0.5)
        self.LCMFront.setOpacity(1.0)

        pxRail = QtGui.QPixmap()
        pxRail.convertFromImage(imgRail)
        self.rail = QtGui.QGraphicsPixmapItem(pxRail)


        self.LCMFront.scale(self.SCALING, self.SCALING)
        self.LCMBack.scale(self.SCALING, self.SCALING)
        self.rail.scale(self.SCALING, self.SCALING)

        self.grscene.addItem(self.rail)
        self.grscene.addItem(self.LCMBack)
        self.grscene.addItem(self.LCMFront)

        # home position is FULLY EXTENDED POT
        self.LCM_HomePos = 0*self.total_inch_to_im_scale
        self.LCMFront_Pos = 0
        self.LCMBack_Pos = 0

        self.LCM_Vert_HomePos = 0*self.total_inch_to_im_scale
        self.LCM_Vert_Pos_Front = 0
        self.LCM_Vert_Pos_Back = 0

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

        subf = self.lc.subscribe("_AF_OM", self.handle_af_om)
        subr = self.lc.subscribe("_AR_OM", self.handle_ar_om)

    def update(self):
        self.LCMFront.setX(self.LCM_HomePos - self.LCMFront_Pos)
        self.LCMBack.setX(self.LCM_HomePos + self.LCMBack_Pos)

        self.LCMFront.setY(self.LCM_Vert_HomePos + self.LCM_Vert_Pos_Front)
        self.LCMBack.setY(self.LCM_Vert_HomePos + self.LCM_Vert_Pos_Back)
    
    def handle_af_om(self, channel, data):
        msg = analog_front_medium_rate_t.decode(data)

        df = msg.lcm_front_gh
        self.LCMFront_Pos = ((df - 42)/25.4)*self.total_inch_to_im_scale

    def handle_ar_om(self, channel, data):
        msg = analog_rear_medium_rate_t.decode(data)

        df = msg.lcm_rear_gh
        self.LCMBack_Pos = ((df - 42)/25.4)*self.total_inch_to_im_scale


if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    with open('../config/simConfig.yaml', 'r') as f:
        config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = LCMVisWidget(config, lc=lc)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
