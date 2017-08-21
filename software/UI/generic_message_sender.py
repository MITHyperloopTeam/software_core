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

#interface stuff
from PyQt4 import QtCore, QtGui, QtOpenGL
import pyqtgraph as pg

#comms stuff
import lcm
from mithl import vectorXf_t
from mithl import trigger_t
from lcm_utils import *

from copy import deepcopy

#read yaml config information
import yaml

class GenericMessageSender(QtGui.QWidget):
    ''' Given a message type, constructs a widget to send that kind of message. 
           lc must be an lcm instance
           message_type must be the message type itself -- not an instance
           default_vals should be a dictionary with each message field mapping to a value for that field
           publish_channel and receive_channel are channels on which to send the message and receive 
              confirmations
           limits_dict supplies numeric limits for float and integer valued fields, as a dictionary keyed by
              field name 
           options_dict is keyed by field names, and valued by lists of strings indicating mode values 
              that that field might take. use this to inform this class how to construct dropdown menus
              for submitting enums
    '''
    def __init__(self, lc, message_type, default_vals, publish_channel, receive_channel, limits_dict={}, options_dict={}, parent=None, name=None, spam_period=None):
        super(GenericMessageSender, self).__init__(parent)

        self.accessMutex = Lock()

        self.message_type_ = message_type
        self.publish_channel_ = publish_channel
        self.receive_channel_ = receive_channel
        self.lc = lc

        self.last_known_config = default_vals
        self.tentative_config = deepcopy(self.last_known_config)

        self.limits_dict_ = limits_dict

        self.options_dict_ = options_dict

        # If spam rate is set, we'll add a button to toggle "active sending"
        # that sends the current entered config periodically
        self.spam_period = spam_period

        self.lc = lc
        if name:
            self.setObjectName(name)
        self.startTime = time.time()

        # for convenience of setup and maintenance, store the
        # label, the interface element setter, and interface element getter
        self.configValueFunctionLabelPairs = []

        mainLayout = QtGui.QVBoxLayout()

        # add dialog for each type in the message
        scrollLayout = QtGui.QGridLayout()

        # Top labels
        row_i = 0
        column_label_layout = QtGui.QHBoxLayout()
        
        field_label = QtGui.QLabel("FIELD NAME")
        scrollLayout.addWidget(field_label, row_i, 0)
        field_label_new = QtGui.QLabel("NEW VALUE")
        scrollLayout.addWidget(field_label_new, row_i, 1)
        field_label_curr = QtGui.QLabel("CURRENT VALUE")
        scrollLayout.addWidget(field_label_curr, row_i, 2)

        row_i += 1

        for field in self.message_type_.__slots__:
            if field == 'utime':
                continue
            this_type = type(eval("self.message_type_()." + field))
            this_label = QtGui.QLabel(field + ":")
            scrollLayout.addWidget(this_label, row_i, 0)

            if field in self.options_dict_.keys():
                # enum field
                comboBox =  QtGui.QComboBox()
                i = 0
                comboLookupDict = {}
                for item in self.options_dict_[field]:
                    num = getattr(self.message_type_, item)
                    comboBox.addItem(str(num) + ": " + item)
                    comboLookupDict[i] = num
                    i += 1
                scrollLayout.addWidget(comboBox, row_i, 1)
                comboLabel = QtGui.QLabel("???")
                scrollLayout.addWidget(comboLabel, row_i, 2)
                self.configValueFunctionLabelPairs.append((field, lambda: comboLookupDict.get(comboBox.currentIndex()), comboLabel))
            elif this_type == type(0):
                # int
                intSpinBox = QtGui.QSpinBox()
                intSpinBox.setSingleStep(1)
                intSpinBox.setRange(0,4096)
                if field in self.limits_dict_.keys():
                    intSpinBox.setRange(self.limits_dict_[field][0], self.limits_dict_[field][1])
                intSpinBox.setValue(default_vals[field])
                scrollLayout.addWidget(intSpinBox, row_i, 1)
                intLabel = QtGui.QLabel("???")
                scrollLayout.addWidget(intLabel, row_i, 2)
                self.configValueFunctionLabelPairs.append((field, intSpinBox.value, intLabel))
                pass
            elif this_type == type(0.0):
                # float
                floatSpinBox = QtGui.QDoubleSpinBox()
                floatSpinBox.setDecimals(5)
                floatSpinBox.setSingleStep(0.01)
                if field in self.limits_dict_.keys():
                    floatSpinBox.setRange(self.limits_dict_[field][0], self.limits_dict_[field][1])
                floatSpinBox.setValue(default_vals[field])
                scrollLayout.addWidget(floatSpinBox, row_i, 1)
                floatLabel = QtGui.QLabel("???")
                scrollLayout.addWidget(floatLabel, row_i, 2)
                self.configValueFunctionLabelPairs.append((field, floatSpinBox.value, floatLabel))
            elif this_type == type([]):
                # list -- not handling for now
                print "Don't know how to handle a list."
            elif this_type == type(False):
                # boolean -- checkbox
                checkbox = QtGui.QCheckBox()
                checkbox.setChecked(default_vals[field])
                scrollLayout.addWidget(checkbox, row_i, 1)
                checkLabel = QtGui.QLabel("???")
                scrollLayout.addWidget(checkLabel, row_i, 2)
                self.configValueFunctionLabelPairs.append((field, checkbox.isChecked, checkLabel))
                pass
            else:
                print "GenericMessageSender got unknown type: ", this_type, " from field ", field, " of message ", self.message_type_

            row_i += 1


        scrollInternalWidget = QtGui.QWidget()
        scrollInternalWidget.setLayout(scrollLayout)
        scrollArea = QtGui.QScrollArea()
        scrollArea.setWidget(scrollInternalWidget)
        scrollArea.setMinimumWidth(scrollInternalWidget.sizeHint().width())
        scrollArea.setSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)

        mainLayout.addWidget(scrollArea)

        # the almighty submit button
        submitButton = QtGui.QPushButton('Submit', self)
        submitButton.clicked.connect(self.handleSubmitButton)
        mainLayout.addWidget(submitButton)

        row_i += 1
        if (self.spam_period):
            self.active_command_mode = False
            self.commandActiveModeToggleBtn = QtGui.QPushButton()
            self.commandActiveModeToggleBtn.setText("ENABLE Active Command")
            self.commandActiveModeToggleBtn.clicked.connect(self.toggleActiveCommandMode)
            self.last_sent_command = time.time() - self.spam_period
            mainLayout.addWidget(self.commandActiveModeToggleBtn)

        self.setLayout(mainLayout)

        # set up default msg
        self.desiredConfigMsg = self.message_type_()
        self.desiredConfigMsg.utime = time.time()*1000*1000
        for pair in self.configValueFunctionLabelPairs:
            setattr(self.desiredConfigMsg, pair[0], pair[1]())    
        self.desiredConfigMutex = Lock()
        
        # just to be sure...
        self.updateAllConfig() 
        self.update()

        configsub = self.lc.subscribe(self.receive_channel_, self.handle_response)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def toggleActiveCommandMode(self):
        if (self.active_command_mode):
            self.commandActiveModeToggleBtn.setText("ENABLE Active Command")
            self.active_command_mode = False    
        else:
            self.active_command_mode = True
            self.commandActiveModeToggleBtn.setText("DISABLE Active Command")

    def update(self):
        self.accessMutex.acquire()
        self.updateAllConfig()
        if (self.spam_period and self.active_command_mode and time.time() - self.last_sent_command > self.spam_period):
            self.last_sent_command = time.time()
            self.submitDesiredConfig()
        self.accessMutex.release()
        
    def updateAllConfig(self):
        # handle spinboxes
        nonmatch_palette = QtGui.QPalette()
        nonmatch_palette.setColor(QtGui.QLabel().backgroundRole(), QtGui.QColor(255, 0, 0, 50))

        match_palette = QtGui.QPalette()
        match_palette.setColor(QtGui.QLabel().backgroundRole(), QtCore.Qt.white)

        approx_equal = lambda a, b, t: abs(a - b) < t
        for pair in self.configValueFunctionLabelPairs:
            if not approx_equal(pair[1](),  self.last_known_config[pair[0]], 0.0001):
                pair[2].setPalette(nonmatch_palette)
            else:
                pair[2].setPalette(match_palette)
            pair[2].setText(str(self.last_known_config[pair[0]]))

    def handleSubmitButton(self):
        msg = self.message_type_()
        msg.utime = time.time()*1000*1000
        for pair in self.configValueFunctionLabelPairs:
            setattr(msg, pair[0], pair[1]())    
        self.desiredConfigMutex.acquire()
        self.desiredConfigMsg = msg
        self.desiredConfigMutex.release()
        self.submitDesiredConfig()

    def submitDesiredConfig(self):
        self.desiredConfigMutex.acquire()
        self.lc.publish(self.publish_channel_, self.desiredConfigMsg.encode())
        self.desiredConfigMutex.release()

    def handle_response(self, channel, data):
        self.accessMutex.acquire()
        msg = self.message_type_.decode(data)
        for key in self.last_known_config.keys():
            try:
                newval = getattr(msg, key)
                if (newval != -2):
                    self.last_known_config[key] = newval
            except Exception as e:
                print "Problem in parsing ", self.message_type_, ": ", e
        self.accessMutex.release()



if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    
    from mithl import braking_low_level_t
    window = GenericMessageSender(lc=lc, message_type=braking_low_level_t, 
        default_vals={
                "utime" : 0,
                "PWREN" : False,
                "PumpMotorPWM" : 0.0,
                "PumpMotorEnable" : False,
                "PumpMotorDRC" : False,
                "OnOffValve" : False,
                "ValvePWM" : 0.0,
                "LSMotorEnable" : False,
                "LSMotorDisable" : True,
                "LSActuatorEnable" : False,
                "LSActuatorDisable" : True
        }, publish_channel="_BRAKE_TEST_DATA_DOWN", receive_channel="_BRAKE_TEST_DATA_UP")
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
