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
from PyQt4 import QtCore, QtGui, QtOpenGL

#comms stuff
import lcm
from mithl import trigger_t
from lcm_utils import *

lc = None

def sendEStop():
    msg = trigger_t()
    msg.utime = time.time()*1000*1000
    lc.publish("_ESTOP", msg.encode())

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    overallWidget = QtGui.QWidget()
    overallLayout = QtGui.QVBoxLayout(overallWidget)

    estopButton = QtGui.QPushButton('ESTOP')
    estopButton.clicked.connect(sendEStop)
    estopButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)

    s = estopButton.styleSheet()
    s += "font: 36pt;"
    s += "background-color: {:s}; color: white;".format("Red")
    estopButton.setStyleSheet(s)

    overallLayout.addWidget(estopButton)


    
    overallWidget.setWindowTitle('Big Red ESTOP')
    overallWidget.show()
    overallWidget.resize(500,600)

    start_lcm(lc)

    sys.exit(app.exec_())