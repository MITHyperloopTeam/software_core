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
from mithl import trigger_t
from mithl import flight_control_high_rate_t
from mithl import battery_status_t
from mithl import adm_status_t, adm_config_t
from mithl import state_t,bac_mode_t
from mithl import spacex_telemetry_state_t

from mithl import analog_front_low_rate_t

from lcm_utils import *

LABEL_DEFAULT_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 0px; border-color: black;"
LABEL_DISPLAY_GOOD_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: green; color: white"
LABEL_DISPLAY_WARN_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: blue; color: white"
LABEL_DISPLAY_PANIC_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: red; color: white"
LAYOUT_PANIC_STYLE_SHEET = "background-color: red"
LAYOUT_GOOD_STYLE_SHEET = "background-color: none"
TITLE_DEFAULT_STYLE_SHEET = "font: 24pt"



def wrapInVTitledItem(name, items):
  box = QtGui.QGroupBox(name)
  box.setAlignment(QtCore.Qt.AlignCenter) 
  boxLayout = QtGui.QVBoxLayout(box)
  for item in items:
    boxLayout.addWidget(item)
  return box
def wrapInHTitledItem(name, items):
  box = QtGui.QGroupBox(name)
  box.setAlignment(QtCore.Qt.AlignCenter) 
  boxLayout = QtGui.QHBoxLayout(box)
  for item in items:
    boxLayout.addWidget(item)
  return box
def spawnBasicLabel(txt = "<>"):
  qs = QtGui.QLabel(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  qs.setAlignment(QtCore.Qt.AlignCenter)
  return qs
def spawnBasicLabelDisplay(txt = "<>"):
  qs = QtGui.QLabel(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
  qs.setAlignment(QtCore.Qt.AlignCenter)
  return qs

def spawnBasicButton(txt = "<>"):
  qs = QtGui.QPushButton(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs
def spawnBasicTextEntry():
  qs = QtGui.QLineEdit('')
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs
def spawnProgressBar(max, min):
  ql = QtGui.QProgressBar()
  ql.setMaximum(max)
  ql.setMinimum(min)
  ql.setOrientation(QtCore.Qt.Vertical)
  return ql

def wrapWithTitleOnLeft(name, items):
  box = QtGui.QWidget()
  boxLayout = QtGui.QHBoxLayout(box)
  boxLayout.addWidget(spawnBasicLabel(name))
  for item in items:
    boxLayout.addWidget(item)
  return box

class PodHealthPanel(QtGui.QWidget):
    ''' Summarizes some high-level pod health information. '''
    def __init__(self, lc=None, parent=None, name=None):
        super(PodHealthPanel, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        hBoxLayout  = QtGui.QHBoxLayout()

        self.watchdogTimeout = 3.0
        self.case_temp_estimate = 12.0

        ## ESTOP
        estopButton = QtGui.QPushButton('ESTOP')
        estopButton.clicked.connect(self.sendEStop)
        estopButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = estopButton.styleSheet()
        s += "font: 36pt;"
        s += "background-color: {:s}; color: white;".format("Red")
        estopButton.setStyleSheet(s)
        hBoxLayout.addWidget(estopButton)

        # MODE and BAC MODE
        modeAndBACModeV = QtGui.QVBoxLayout()

        self.fsmStateView = spawnBasicLabelDisplay("???")
        fsmStateBox = wrapWithTitleOnLeft("FSM State:", [self.fsmStateView])    
        modeAndBACModeV.addWidget(fsmStateBox)
        self.lastStateMsg = time.time() - self.watchdogTimeout
        self.state = None
        self.lc.subscribe("FSM_STATE", self.handle_state_t)


        self.bacStateView = spawnBasicLabelDisplay("???")
        bacStateBox = wrapWithTitleOnLeft("BAC Mode:", [self.bacStateView])
        modeAndBACModeV.addWidget(bacStateBox)
        self.lastBACModeMsg = time.time() - self.watchdogTimeout
        self.bacMode = None
        self.lc.subscribe("_BAC_MODE_STATE", self.handle_bac_mode_t)

        hBoxLayout.addLayout(modeAndBACModeV)

        # ADM Modes
        radmModeV = QtGui.QVBoxLayout()

        self.wheelControllerModeView = spawnBasicLabelDisplay("UNKNOWN")
        wheelControllerModeBox = wrapWithTitleOnLeft("RADM Wheel State:", [self.wheelControllerModeView])
        radmModeV.addWidget(wheelControllerModeBox)


        self.clampControllerModeView = spawnBasicLabelDisplay("UNKNOWN")
        clampControllerModeBox = wrapWithTitleOnLeft("RADM Clamp State:", [self.clampControllerModeView])
        radmModeV.addWidget(clampControllerModeBox)


        hBoxLayout.addLayout(radmModeV)

        telemetryLayout = QtGui.QVBoxLayout()
        self.telemetry_status = QtGui.QLabel("UNKNOWN")
        self.telemetry_status.setAlignment(QtCore.Qt.AlignCenter)
        telemetry_status_box = wrapInHTitledItem("SpaceX Telemetry Status", [self.telemetry_status])
        telemetryLayout.addWidget(telemetry_status_box)
        self.last_heard_spacex_status = time.time() - self.watchdogTimeout
        self.lc.subscribe("SPACEX_RELAY_STATE", self.handle_spacex_relay_state)
        faultClearButton = QtGui.QPushButton("Clear Telemetry Fault")
        faultClearButton.clicked.connect(self.sendFaultClear)
        faultClearButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        s = faultClearButton.styleSheet()
        s += "font: 16pt;"
        faultClearButton.setStyleSheet(s)
        telemetryLayout.addWidget(faultClearButton)
        hBoxLayout.addLayout(telemetryLayout)

        # Pod stop budget
        self.podStoppedLabel = spawnBasicLabelDisplay("UNKNOWN")

        hBoxLayout.addWidget(wrapInHTitledItem("Pod Stopped", [self.podStoppedLabel]))
        self.setLayout(hBoxLayout)
        self.stopped = False
        self.lastStoppedMsgTime = time.time() - self.watchdogTimeout
        self.lc.subscribe("_FC_OH", self.handle_flight_control_high_rate_t)

        # 24V battery info
        lowBatteryGrid = QtGui.QGridLayout()
        self.lowBatteryVoltageView = spawnBasicLabelDisplay("UNKNOWN")
        self.lowBatteryCurrentView = spawnBasicLabelDisplay("UNKNOWN")
        self.lowBatteryTempView = spawnBasicLabelDisplay("UNKNOWN")
        #self.lowBatteryTimeEst = spawnBasicLabelDisplay("UNKNOWN")
        lowBatteryGrid.addWidget(self.lowBatteryVoltageView, 0, 0, 1, 1)
        lowBatteryGrid.addWidget(self.lowBatteryCurrentView, 0, 1, 1, 1)
        lowBatteryGrid.addWidget(self.lowBatteryTempView, 1, 0, 1, 1)
        #lowBatteryGrid.addWidget(self.lowBatteryTimeEst, 1, 1, 1, 1)
        self.lowBatteryGridObject = QtGui.QWidget();
        self.lowBatteryGridObject.setLayout(lowBatteryGrid);
        lowBatteryBox = wrapInHTitledItem("24V Battery:", [self.lowBatteryGridObject])
        hBoxLayout.addWidget(lowBatteryBox)
        self.lastLowBatteryMsg = None
        self.lastLowBatteryMsgTime = time.time() - self.watchdogTimeout
        self.lc.subscribe("_AF_BATTERY", self.handle_battery_status_t)

        # 36V battery info
        highBatteryGrid = QtGui.QGridLayout()
        self.highBatteryVoltageView = spawnBasicLabelDisplay("UNKNOWN")
        self.highBatteryTempView = spawnBasicLabelDisplay("UNKNOWN")
        highBatteryGrid.addWidget(self.highBatteryVoltageView, 0, 0, 1, 1)
        highBatteryGrid.addWidget(self.highBatteryTempView, 1, 0, 1, 1)
        self.highBatteryGridObject = QtGui.QWidget();
        self.highBatteryGridObject.setLayout(highBatteryGrid);
        highBatteryBox = wrapInHTitledItem("36V Battery:", [self.highBatteryGridObject])
        hBoxLayout.addWidget(highBatteryBox)

        # not really used anymore
        self.lastHighBatteryMsg = None
        self.lastHighBatteryMsgTime = time.time() - self.watchdogTimeout
        self.lc.subscribe("_AR_BATTERY", self.handle_battery_status_t)

        # Used to infer 36V battery status instead
        self.lastWheelStatusMsg = None
        self.lastWheelStatusMsgTime = time.time() - self.watchdogTimeout
        self.lastClampStatusMsg = None
        self.lastClampStatusMsgTime = time.time() - self.watchdogTimeout
        self.lc.subscribe("_RADM_CLAMP_STATUS", self.handle_adm_status_t)
        self.lc.subscribe("_RADM_WHEEL_STATUS", self.handle_adm_status_t)

        self.lastWheelConfigMsg = None
        self.lastWheelConfigMsgTime = time.time() - self.watchdogTimeout
        self.lastClampConfigMsg = None
        self.lastClampConfigMsgTime = time.time() - self.watchdogTimeout
        self.lc.subscribe("_RADM_WHEEL_CURCONF", self.handle_adm_config_t)
        self.lc.subscribe("_RADM_CLAMP_CURCONF", self.handle_adm_config_t)

        self.lastAnalogFrontLowRateMsgTime = time.time() - self.watchdogTimeout
        self.lastAnalogFrontLowRateMsg = None
        self.lc.subscribe("_AF_OL", self.handle_analog_front_low_rate_t)


        self.setLayout(hBoxLayout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        ### FSM STATE
        if (time.time() - self.lastStateMsg > self.watchdogTimeout or self.state is None):
            self.fsmStateView.setText("???")
            self.fsmStateView.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
        elif self.state is "ESTOP":
            self.fsmStateView.setText(self.state)
            self.fsmStateView.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
        else:
            self.fsmStateView.setText(self.state)
            self.fsmStateView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)

        ### BAC Mode
        if (time.time() - self.lastBACModeMsg > self.watchdogTimeout or self.bacMode is None):
            self.bacStateView.setText("???")
            self.bacStateView.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
        elif self.bacMode is "ESTOP":
            self.bacStateView.setText(self.bacMode)
            self.bacStateView.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
        else:
            self.bacStateView.setText(self.bacMode)
            self.bacStateView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)

        ### ADM Clamp Config
        if (time.time() - self.lastClampConfigMsgTime > self.watchdogTimeout or self.lastClampConfigMsg is None or self.lastClampConfigMsg.estop < 0):
            self.clampControllerModeView.setText("???")
            self.clampControllerModeView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
        elif self.lastClampConfigMsg.estop == 1:
            self.clampControllerModeView.setText("ESTOP")
            self.clampControllerModeView.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
        else:
            self.clampControllerModeView.setText("ARM")
            self.clampControllerModeView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)

        ### ADM Wheel Config
        if (time.time() - self.lastWheelConfigMsgTime > self.watchdogTimeout or self.lastWheelConfigMsg is None or self.lastWheelConfigMsg.estop < 0):
            self.wheelControllerModeView.setText("???")
            self.wheelControllerModeView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
        elif self.lastClampConfigMsg.estop == 1:
            self.wheelControllerModeView.setText("ESTOP")
            self.wheelControllerModeView.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
        else:
            self.wheelControllerModeView.setText("ARM")
            self.wheelControllerModeView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)

        if (time.time() - self.last_heard_spacex_status < self.watchdogTimeout):
            msg = spacex_telemetry_state_t()
            if (self.spacex_status == msg.STATUS_FAULT):
                self.telemetry_status.setText("FAULT")
                self.telemetry_status.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
            elif (self.spacex_status == msg.STATUS_IDLE):
                self.telemetry_status.setText("IDLE")
                self.telemetry_status.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
            elif (self.spacex_status == msg.STATUS_READY):
                self.telemetry_status.setText("READY")
                self.telemetry_status.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
            elif (self.spacex_status == msg.STATUS_PUSHING):
                self.telemetry_status.setText("PUSHING")
                self.telemetry_status.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
            elif (self.spacex_status == msg.STATUS_COAST):
                self.telemetry_status.setText("COAST")
                self.telemetry_status.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
            elif (self.spacex_status == msg.STATUS_BRAKING):
                self.telemetry_status.setText("BRAKING")
                self.telemetry_status.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
        else:
            self.telemetry_status.setText("UNKNOWN")
            self.telemetry_status.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)

        ### Stopping Detector
        if (time.time() - self.lastStoppedMsgTime < self.watchdogTimeout):
            self.podStoppedLabel.setText("Timed out")
            self.podStoppedLabel.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
        elif (self.stopped):
            self.podStoppedLabel.setText("Stopped")
            self.podStoppedLabel.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
        else:
            self.podStoppedLabel.setText("Moving")
            self.podStoppedLabel.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)

        if (self.lastLowBatteryMsg and time.time() - self.lastLowBatteryMsgTime < self.watchdogTimeout):
            self.lowBatteryGridObject.setStyleSheet(LAYOUT_GOOD_STYLE_SHEET)
            rangeCheckQueue = (
                # display, value, good_min, good_max, warn_min, warn_max
                (self.lowBatteryTempView, self.lastLowBatteryMsg.internal_temp, 0., 50., 50., 60., " *C"),
                (self.lowBatteryVoltageView, float(self.lastLowBatteryMsg.pack_voltage), 27., 30., 24., 27., " V"),
                (self.lowBatteryCurrentView, self.lastLowBatteryMsg.current, 0., 10., 10., 20., " Amps"),
            )
            for rangeCheck in rangeCheckQueue:
                if (self.lastLowBatteryMsg.battery_comms_lock):
                    rangeCheck[0].setText(("%4.2f" + rangeCheck[6]) % rangeCheck[1])
                    if (rangeCheck[1] >= rangeCheck[2] and rangeCheck[1] <= rangeCheck[3]):
                        rangeCheck[0].setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
                    elif (rangeCheck[1] >= rangeCheck[4] and rangeCheck[1] <= rangeCheck[5]):
                        rangeCheck[0].setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
                    else:
                        rangeCheck[0].setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
                else:
                    rangeCheck[0].setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
                    rangeCheck[0].setText("???")     
        else:
            self.lowBatteryGridObject.setStyleSheet(LAYOUT_PANIC_STYLE_SHEET)


        if (self.lastClampStatusMsg and time.time() - self.lastClampStatusMsgTime < self.watchdogTimeout):
            if self.lastClampStatusMsg.voltage[1] < -1:
                self.highBatteryVoltageView.setText("???")
                self.highBatteryVoltageView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
            elif self.lastClampStatusMsg.voltage[1] > 30:
                self.highBatteryVoltageView.setText("%.4f V" % self.lastClampStatusMsg.voltage[1])
                if (self.lastClampStatusMsg.voltage[1] < 38):
                    self.highBatteryVoltageView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
                else:
                    self.highBatteryVoltageView.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
            else:
                self.highBatteryVoltageView.setText("%.4f V (OFF?)" % self.lastClampStatusMsg.voltage[1])
                self.highBatteryVoltageView.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)

        else:
            self.highBatteryVoltageView.setText("???")
            self.highBatteryVoltageView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)

        if (self.lastAnalogFrontLowRateMsg and time.time() - self.lastAnalogFrontLowRateMsgTime < self.watchdogTimeout):
                self.highBatteryTempView.setText("%.4f *C" % self.lastAnalogFrontLowRateMsg.front_battery_temp)
                if self.lastAnalogFrontLowRateMsg.front_battery_temp > 0 and self.lastAnalogFrontLowRateMsg.front_battery_temp < 60:
                    self.highBatteryTempView.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
                else:
                    self.highBatteryTempView.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
        else:
            self.highBatteryTempView.setText("???")
            self.highBatteryTempView.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)


    def sendEStop(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish("_ESTOP", msg.encode())

    def sendFaultClear(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish("CLEAR_FAULT", msg.encode())

    def handle_analog_front_low_rate_t(self, channel, data):
        msg = analog_front_low_rate_t.decode(data)
        self.lastAnalogFrontLowRateMsgTime = time.time()
        self.lastAnalogFrontLowRateMsg = msg

    def handle_adm_status_t(self, channel, data):
        msg = adm_status_t.decode(data)
        if (channel == "_RADM_CLAMP_STATUS"):
            self.lastClampStatusMsg = msg
            self.lastClampStatusMsgTime = time.time()
            self.case_temp_estimate = self.case_temp_estimate*0.9 + self.lastClampStatusMsg.case_temp*0.1
        if (channel == "_RADM_WHEEL_STATUS"):
            self.lastWheelStatusMsg = msg
            self.case_temp_estimate = self.case_temp_estimate*0.9 + self.lastWheelStatusMsg.case_temp*0.1
            self.lastWheelStatusMsgTime = time.time()

    def handle_adm_config_t(self, channel, data):
        msg = adm_config_t.decode(data)
        if (channel == "_RADM_CLAMP_CURCONF"):
            self.lastClampConfigMsg = msg
            self.lastClampConfigMsgTime = time.time()
        if (channel == "_RADM_WHEEL_CURCONF"):
            self.lastWheelConfigMsg = msg
            self.lastWheelConfigMsgTime = time.time()

    def handle_battery_status_t(self, channel, data):
        msg = battery_status_t.decode(data)
        if (channel == "_AR_BATTERY"):
            self.lastLowBatteryMsg = msg
            self.lastLowBatteryMsgTime = time.time()
        else:
            self.lastHighBatteryMsg = msg
            self.lastHighBatteryMsgTime = time.time()

    def handle_flight_control_high_rate_t(self, channel, data):
        msg = flight_control_high_rate_t.decode(data)
        self.stopped = msg.stopped

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

    def handle_state_t(self, channel, data):
        msg = state_t.decode(data)

        poss = {
            0: "ESTOP",
            1: "SHUTDOWN",
            2: "SOFT_STOP",
            10: "ARMING",
            11: "ARM",
            12: "LAUNCH",
            13: "FLIGHT",
            20: "PRE_DRIVE",
            21: "DRIVE",
            99: "TELEOP"
        }
        self.lastStateMsg = time.time()
        if msg.currentState in poss.keys():
            self.state = poss[msg.currentState]
        else:
            self.state = None
            print "Unknown state received, ignoring"

    def handle_bac_mode_t(self, channel, data):
        msg = bac_mode_t.decode(data)

        self.lastBACModeMsg = time.time()
        if msg.mode == 0:
            self.bacMode = "ESTOP"
        elif msg.mode == 1:
            self.bacMode = "TELEOP"
        elif msg.mode == 2:
            self.bacMode = "AUTO"
        else:
            self.bacMode = None
            print "Unknown bacmode received, ignoring"

    def handle_spacex_relay_state(self, channel, data):
        msg = spacex_telemetry_state_t.decode(data)
        self.spacex_status = msg.status
        self.last_heard_spacex_status = time.time()

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = PodHealthPanel(lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())
