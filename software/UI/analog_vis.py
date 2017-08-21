# the python stuff
import sys
import math
import signal

# numerics
import numpy as np

# the interface stuff
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

# the messaging stuff
import lcm
from mithl import vectorXf_t
from lcm_utils import *


class AnalogPlotWidget(QtGui.QWidget):
    ''' Displays analog data and sends analog on/off commands. '''
    def __init__(self, indices, title="Analog plotter", histLength=100, dataRange=None, color=None, lc=None, channel="VEC", parent=None, name=None):
        super(AnalogPlotWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)


        self.plot = pg.PlotWidget(title=title)
        self.plot.hideAxis("bottom")
        self.curves = []
        self.indices = indices
        self.ncurves = len(indices)
        self.dataRange = dataRange

        for i in range(self.ncurves):
            if not color:
                self.curves.append(self.plot.plot([0], [0], pen=(i, self.ncurves)))
            else:
                self.curves.append(self.plot.plot([0], [0], pen=pg.mkPen(color)))

        self.ptr = 0
        self.histLength = histLength
        self.good_ptr = 0

        self.analogDataHist = []
        self.timeDataHist = np.zeros(self.histLength)
        for i in range(self.ncurves):
            self.analogDataHist.append(np.zeros(self.histLength))
        
        vBoxLayout  = QtGui.QVBoxLayout()
        vBoxLayout.addWidget(self.plot)
        self.setLayout(vBoxLayout)

        analogchannel_sub = self.lc.subscribe(channel, self.handle_vectorXf_t)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        for i in range(self.ncurves):
            self.curves[i].setData(self.timeDataHist[0:self.good_ptr], self.analogDataHist[i][0:self.good_ptr])
        if self.dataRange:
            self.plot.setYRange(self.dataRange[0], self.dataRange[1])
        #self.plot.setXRange(np.min(self.timeDataHist[0:self.good_ptr], np.max(self.timeDataHist[0:self.good_ptr])))
        #self.plot.setXRange(0, 1000)

    def handle_vectorXf_t(self, channel, data):
        msg = vectorXf_t.decode(data)
        self.timeDataHist[self.ptr] = float(msg.utime)/1000000

        for i, ind in enumerate(self.indices):
            if (ind < msg.rows):
                if self.ptr == self.histLength-1 and self.good_ptr == self.histLength-1:
                    # todo: this is horrendously inefficient
                    self.analogDataHist[i] = np.roll(self.analogDataHist[i], -1)
                self.analogDataHist[i][self.ptr] = msg.data[ind]
        if self.ptr == self.histLength-1 and self.good_ptr == self.histLength-1:
            self.timeDataHist = np.roll(self.timeDataHist, -1)

        self.good_ptr = max(self.ptr, self.good_ptr)
        self.ptr = min(self.ptr+1,  self.histLength-1)



class AnalogVisWidget(QtGui.QWidget):
    ''' Displays analog data and sends analog on/off commands. '''
    def __init__(self, indices, channel="VEC", lc=None, parent=None, name=None):
        super(AnalogVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        self.analogLabels =   []
        self.nIndices = len(indices)
        self.indices = indices
        for i in range(self.nIndices):
            self.analogLabels.append(QtGui.QLabel("A" + str(i) + ": _____"))

        vBoxLayout  = QtGui.QVBoxLayout()
        for label in self.analogLabels:
            vBoxLayout.addWidget(label)
        self.setLayout(vBoxLayout)

        analogchannel_sub = self.lc.subscribe(channel, self.handle_vectorXf_t)

    def handle_vectorXf_t(self, channel, data):
        msg = vectorXf_t.decode(data)
        for i, ind in enumerate(self.indices):
            if (ind < msg.rows):
                self.analogLabels[i].setText("A" + str(ind) + ": " + str(msg.data[ind]))

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = AnalogPlotWidget(indices=range(11), channel="VEC", histLength=100, lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())
