# the python stuff
import sys
import math
import signal
import time

# numerics
import numpy as np

# the interface stuff
from PyQt4 import QtCore, QtGui

# the messaging stuff
import lcm
from mithl import string_t
from lcm_utils import *

class ZCMRoundTripWidget(QtGui.QWidget):
    ''' Displays analog data and sends analog on/off commands. '''
    def __init__(self, lc=None, parent=None, name=None):
        super(ZCMRoundTripWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        self.textBox = QtGui.QLineEdit("<>")
        self.btn = QtGui.QPushButton("Send")
        self.btn.clicked.connect(self.btn_press_handler)
        self.textlabel = QtGui.QLabel("Last received: <>")
        self.rttlabel = QtGui.QLabel("RTT: ")

        self.lastSendTime = 0
        self.starttime = time.time()

        vBoxLayout  = QtGui.QVBoxLayout()
        vBoxLayout.addWidget(self.textBox)
        vBoxLayout.addWidget(self.btn)
        vBoxLayout.addWidget(self.textlabel)
        vBoxLayout.addWidget(self.rttlabel)
        self.setLayout(vBoxLayout)

        analogchannel_sub = self.lc.subscribe("STR_REC", self.handle_string_t)

    def btn_press_handler(self):
        sendtime = time.time()
        msg = string_t()
        msg.utime = int((sendtime - self.starttime)* 1000000)
        msg.str = str(self.textBox.text())
        self.lc.publish("STR_SEND", msg.encode())
        self.lastSendTime = sendtime

    def handle_string_t(self, channel, data):
        msg = string_t.decode(data)
        self.textlabel.setText("Last received: " + msg.str)
        self.rttlabel.setText("RTT: " + str( time.time() - self.lastSendTime ))

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = ZCMRoundTripWidget(lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())
