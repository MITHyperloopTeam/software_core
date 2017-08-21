# the python stuff
import sys
import math
import signal
from threading import Lock

# numerics
import numpy as np

# the interface stuff
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

# the messaging stuff
import lcm
from mithl import vectorXf_t
from lcm_utils import *


class DataPlotWidget(QtGui.QWidget):
    ''' Helps display data in a nice-looking way'''
    def __init__(self, ncurves, title="Data plotter", histLength=100, dataRange=None, color=None, parent=None, alsoNumeric=False, avgWindow=0.0, levelLines=None, name=None, runInBackground=False):
        super(DataPlotWidget, self).__init__(parent)
        
        if (name):
            self.setObjectName(name)

        self.runInBackground = runInBackground
        self.accessMutex = Lock()

        self.curves = []
        self.ncurves = ncurves
        self.dataRange = dataRange
        self.alsoNumeric = alsoNumeric
        self.avgWindow = avgWindow
        if (self.avgWindow > 0.0):
            fulltitle = title + " , Low-Pass RC=" + ("%.2f"%avgWindow)
            self.avgs = [0.0]*self.ncurves
            self.lastGotPoint = [0.0]*self.ncurves
        else:
            fulltitle = title
        self.title = title
        self.plot = pg.PlotWidget(title=fulltitle)
        self.plot.hideAxis("bottom")

        if levelLines:
            for line in levelLines:
                line = pg.InfiniteLine(pos=line, angle=0)
                self.plot.addItem(line)

        for i in range(self.ncurves):
            if not color:
                self.curves.append(self.plot.plot([0], [0], pen=(i, self.ncurves)))
            else:
                self.curves.append(self.plot.plot([0], [0], pen=pg.mkPen(color)))

        self.ptr = [0] * self.ncurves
        self.histLength = histLength
        self.good_ptr = [0] * self.ncurves

        self.dataHist = []
        self.timeDataHist = np.zeros((self.histLength, self.ncurves))
        for i in range(self.ncurves):
            self.dataHist.append(np.zeros(self.histLength))
        
        vBoxLayout  = QtGui.QVBoxLayout()
        vBoxLayout.addWidget(self.plot)

        if (self.alsoNumeric):
            self.labels = []
            self.labelText = ["???"]*self.ncurves
            for i in range(self.ncurves):
                self.labels.append(QtGui.QLabel("Val: ???"))
                vBoxLayout.addWidget(self.labels[-1])

        self.setLayout(vBoxLayout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        self.accessMutex.acquire()
        for i in range(self.ncurves):
            self.curves[i].setData(self.timeDataHist[0:self.good_ptr[i], i], self.dataHist[i][0:self.good_ptr[i]])
        if self.dataRange:
            self.plot.setYRange(self.dataRange[0], self.dataRange[1])

        if (self.alsoNumeric):
            for i in range(self.ncurves):
                self.labels[i].setText(self.labelText[i])

        #self.plot.setXRange(np.min(self.timeDataHist[0:self.good_ptr], np.max(self.timeDataHist[0:self.good_ptr])))
        #self.plot.setXRange(0, 1000)
        self.accessMutex.release()

    def autoRangeY(self):
        minY = 0
        maxY = 0
        for i in range(self.ncurves):
            minY = min(minY, np.min(self.dataHist[i][0:self.good_ptr[i]]))
            maxY = max(maxY, np.max(self.dataHist[i][0:self.good_ptr[i]]))
        self.plot.setYRange(minY, maxY)

    def addDataPoint(self, i, t, x):
        self.accessMutex.acquire()
        self.timeDataHist[self.ptr, i] = t 

        # Do the running average
        if (self.avgWindow > 0):
            dt = t - self.lastGotPoint[i]
            self.lastGotPoint[i] = t
            alpha = dt / (dt + self.avgWindow)    
            self.avgs[i] = x * alpha + (1.0 - alpha) * self.avgs[i]
            x = self.avgs[i]    

        if self.ptr[i] == self.histLength-1 and self.good_ptr[i] == self.histLength-1:
            # todo: this is horrendously inefficient
            self.dataHist[i] = np.roll(self.dataHist[i], -1)
        self.dataHist[i][self.ptr[i]] = x

        if self.ptr[i] == self.histLength-1 and self.good_ptr[i] == self.histLength-1:
            self.timeDataHist[:, i] = np.roll(self.timeDataHist[:, i], -1)

        self.good_ptr[i] = max(self.ptr[i], self.good_ptr[i])
        self.ptr[i] = min(self.ptr[i]+1,  self.histLength-1)

        if (self.alsoNumeric):
            # update label text
            # (don't touch actual GUI state outside of main thread)
            if (self.avgWindow > 0):
                self.labelText[i] = self.title + " #" + str(i) + ": " + str(self.avgs[i])
            else:
                self.labelText[i] = self.title + " #" + str(i) + ": " + str(x)


        self.accessMutex.release()
